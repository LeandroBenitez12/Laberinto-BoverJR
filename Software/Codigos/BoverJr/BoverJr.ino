#include <PID.h>
#include <EngineController.h>
#include <DistanceSensors.h>
#include <Button.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//debug
#define TICK_DEBUG_ALL 500
#define DEBUG_STATUS 0
#define DEBUG_SENSORS 0
#define DEBUG_PID 0
unsigned long currentTimePID = 0;
unsigned long currentTimeDebugAll = 0;

//Motores
#define PIN_RIGHT_ENGINE_IN1 26
#define PIN_RIGHT_ENGINE_IN2 27
#define PIN_LEFT_ENGINE_IN1 18
#define PIN_LEFT_ENGINE_IN2 19
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4

//Sharps
#define PIN_SHARP_RIGHT 25
#define PIN_SHARP_LEFT 35
#define PIN_SHARP_FRONT 33
float rightDistance;
float leftDistance;
float frontDistance;
#define MAX_FRONT_DISTANCE 10
#define MAX_SIDE_DISTANCE 20

//veocidades motores pwm
int speedRightPID = 200;
int speedLeftPID = 200;
int averageSpeedRight = 200;
int averageSpeedLeft = 200;
#define TURN_SPEED 200

// tick de delay
#define TICK_TURN_90 180
#define TICK_TURN_180 400
#define TICK_POST_TURN 200
#define TICK_ANT_TURN 120
#define TICK_IGNORE_TURN 200
#define DELAY_STOP 500

// variables pid
double kp = 0.3;
double kd = 0.14;
double setPoint;
float gananciaPID;
double TICK_PID = 20;

//Boton
#define PIN_BUTTON_START 32

IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *Bover = new EngineController(rightEngine, leftEngine);
Isensor *SharpRigh = new Sharp_GP2Y0A21(PIN_SHARP_RIGHT);
Isensor *SharpLeft = new Sharp_GP2Y0A21(PIN_SHARP_LEFT);
Isensor *SharpFront = new Sharp_GP2Y0A21(PIN_SHARP_FRONT);
Pid *PID = new Pid(kp, kd, setPoint, TICK_PID);
Button *buttonStart = new Button(PIN_BUTTON_START);

void SensorsRead()
{
  
  frontDistance = SharpFront->SensorRead();
  rightDistance = SharpRigh->SensorRead();
  leftDistance = SharpLeft->SensorRead();
}

void printPID()
{
  if (millis() > currentTimePID + TICK_DEBUG_ALL)
  {
    currentTimePID = millis();
    SerialBT.println("__");
    SerialBT.print("Ganancia PID: ");
    SerialBT.println(gananciaPID);
    SerialBT.print("speedRight: ");
    SerialBT.print(speedRightPID);
    SerialBT.print(" || speedLeft: ");
    SerialBT.println(speedLeftPID);
    SerialBT.println("__");
  }
}

void printSensors()
{
  SerialBT.print("frontDistance: ");
  SerialBT.print(frontDistance);
  SerialBT.print(" || rightDistance: ");
  SerialBT.print(rightDistance);
  SerialBT.print(" || leftDistance: ");
  SerialBT.println(leftDistance);

}

void turnRight(){
  Bover->Right(TURN_SPEED);
  delay(TICK_TURN_90);
}

void turnLeft(){
  Bover->Left(TURN_SPEED);
  delay(TICK_TURN_90);
}

void postTurn(){
  Bover->Forward(TURN_SPEED);
  delay(TICK_POST_TURN);
}
void antTurn(){
  Bover->Forward(TURN_SPEED);
  delay(TICK_ANT_TURN);
}

void fullTurn(){
  Bover->Right(TURN_SPEED);
  delay(TICK_TURN_180);
} 

void ignoreTurn(){
  Bover->Forward(TURN_SPEED);
  delay(TICK_IGNORE_TURN);
}

enum movement
{
  STANDBY,
  CONTINUE,
  STOP,
  RIGHT_TURN,
  LEFT_TURN,
  FULL_TURN,
  POST_TURN,
  IGNORE_TURN,
  ANT_TURN
};
int movement = STANDBY;

void movementLogic()
{
  switch (movement)
  {
  case STANDBY:
  {
  
    Bover->Stop();
    if(buttonStart->GetIsPress())
    {
      delay(1000);
      movement = CONTINUE;
    }
    break;
  }

  case CONTINUE:
  {
    float input = rightDistance - leftDistance;
    gananciaPID = PID->ComputePid(input);
    speedRightPID = (averageSpeedRight + (gananciaPID));
    speedLeftPID = (averageSpeedLeft - (gananciaPID));
    Bover->Forward(speedRightPID, speedLeftPID);

    if (frontDistance < MAX_FRONT_DISTANCE) movement = STOP;
    if (rightDistance > MAX_SIDE_DISTANCE) movement = ANT_TURN;
    if (leftDistance > MAX_SIDE_DISTANCE) movement = IGNORE_TURN;

    break;
  }

  case STOP:
  {
    Bover->Stop();
    if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = LEFT_TURN;
    if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance <= MAX_SIDE_DISTANCE) movement = FULL_TURN;
    delay(DELAY_STOP);

    break;
  }

  case RIGHT_TURN:
  {
    turnRight();
    Bover->Stop();
    delay(DELAY_STOP);
    movement = POST_TURN;
    break;
  }

  case LEFT_TURN:
  {
    turnLeft();
    Bover->Stop();
    delay(DELAY_STOP);
    movement = POST_TURN;
    break;
  }

  case FULL_TURN:
  {
    fullTurn();
    Bover->Stop();
    delay(DELAY_STOP);
    movement = POST_TURN;
    break;
  }

  case POST_TURN:
  {
    postTurn();
    Bover->Stop();
    delay(DELAY_STOP);
    movement = CONTINUE;
    break;
  }

  case ANT_TURN:
  {
    antTurn();
    Bover->Stop();
    delay(DELAY_STOP);
    movement = RIGHT_TURN;
    break;
  }

  case IGNORE_TURN:
  {
    ignoreTurn();
    Bover->Stop();
    delay(DELAY_STOP);
    movement = CONTINUE;
    break;
  }
  }
}

void printStatus(){
  String state = "";
  switch (movement)
  {
    case STANDBY: state = "STANDBY";
    break;
    case CONTINUE: state = "CONTINUE";
    break;
    case STOP: state = state = "STOP"; 
    break;
    case RIGHT_TURN: state = "RIGHT TURN"; 
    break;
    case LEFT_TURN: state = "LEFT TURN"; 
    break;
    case FULL_TURN: state = "FULL TURN"; 
    break;
    case POST_TURN: state = "POST TURN";
    break;
    case ANT_TURN: state = "ANT TURN";
    break;
    case IGNORE_TURN: state = "IGNORE_TURN";
    break;
  }
  SerialBT.print("State: ");
  SerialBT.println(state);
}

void printAll(){
  if (millis() > currentTimeDebugAll + TICK_DEBUG_ALL)
  {
    currentTimeDebugAll = millis();
    if (DEBUG_SENSORS) printSensors();
    if (DEBUG_PID) printPID();
    if (DEBUG_STATUS) printStatus();
  }
}
void setup() 
{
  SerialBT.begin("Bover");
}

void loop() 
{
  movementLogic();
  printStatus();
}