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
#define TICK_DEBUG 1500
#define DEBUG_STATUS 1
#define DEBUG_SENSORS 1
#define DEBUG_PID 1
unsigned long currentTimePID = 0;
unsigned long currentTimeSensors = 0;

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
#define MAX_FRONT_DISTANCE 20
#define MAX_SIDE_DISTANCE 12
#define MAX_DISTANCE_ANT_TURN 25

//veocidades motores pwm
int speedRight = 80;
int speedLeft = 80;
int averageSpeed = 100;

//variables pid
double kp = 1.2;
double kd = 0;
double setPoint;
float gananciaPID;
double TICK_PID = 50;

//Boton
#define PIN_BUTTON_START 32
bool start = 0;

IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *Bover = new EngineController(rightEngine, leftEngine);
Isensor *SharpRigh = new Sharp_GP2Y0A21(PIN_SHARP_RIGHT);
Isensor *SharpLeft = new Sharp_GP2Y0A21(PIN_SHARP_LEFT);
Isensor *SharpFront = new Sharp_GP2Y0A21(PIN_SHARP_FRONT);
Button *buttonStart = new Button(PIN_BUTTON_START);
Pid *PID = new Pid(kp, kd, setPoint, TICK_PID);

void SensorsRead()
{
  frontDistance = SharpFront->SensorRead();
  rightDistance = SharpRigh->SensorRead();
  leftDistance = SharpLeft->SensorRead();
}

void printPID()
{
  if (millis() > currentTimePID + TICK_DEBUG)
  {
    currentTimePID = millis();
    SerialBT.print("PID: ");
    SerialBT.println(gananciaPID);
    SerialBT.print("speedRight: ");
    SerialBT.print(speedRight);
    SerialBT.print(" || speedLeft: ");
    SerialBT.println(speedLeft);
  }
}

void printSensors()
{
  if (millis() > currentTimeSensors + TICK_DEBUG)
  {
    currentTimeSensors = millis();
    SerialBT.print("frontDistance: ");
    SerialBT.print(frontDistance);
    SerialBT.print("rightDistance: ");
    SerialBT.print(rightDistance);
    SerialBT.print(" || leftDistance: ");
    SerialBT.println(leftDistance);
  }
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
    Serial.print("Que pared Seguir( '0' = IZQ/ '1' = DER):   ");
    bool  = SerialBT.read();
    Serial.print("INICIAR? ");
    bool stateStart = SerialBT.read();
    if (stateStart) movement = CONTINUE;
    break;
  }

  case CONTINUE:
  {
    float input = rightDistance - leftDistance;
    gananciaPID = PID->ComputePid(input);
    if(DEBUG_PID) printPID();
    speedRight = (averageSpeed - (gananciaPID));
    speedLeft = (averageSpeed + (gananciaPID));
    Bover->Forward(speedRight, speedLeft);

    if (frontDistance < MAX_FRONT_DISTANCE) movement = STOP;
    if (rightDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN) movement = ANT_TURN;
    if (leftDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN) movement = IGNORE_TURN;
    break;
  }

  case STOP:
  {
    Bover->Stop();
    delay(500);
    if (rightDistance > MAX_SIDE_DISTANCE && leftDistance <= MAX_SIDE_DISTANCE) movement = RIGHT_TURN;
    if (rightDistance > MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = RIGHT_TURN;
    if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = LEFT_TURN;
    if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance <= MAX_SIDE_DISTANCE) movement = FULL_TURN;
    break;
  }

  case RIGHT_TURN:
  {
    turnRight();
    Bover->Stop();
    delay(200);
    movement = POST_TURN;
    break;
  }

  case LEFT_TURN:
  {
    turnLeft();
    Bover->Stop();
    delay(200);
    movement = POST_TURN;
    break;
  }

  case FULL_TURN:
  {
    fulTurn();
    Bover->Stop();
    delay(200);
    movement = POST_TURN;
    break;
  }

  case POST_TURN:
  {
    posTurn();
    Bover->Stop();
    delay(200);
    movement = CONTINUE;
    break;
  }

  case ANT_TURN:
  {
    antTurn();
    Bover->Stop();
    delay(200);
    movement = RIGHT_TURN;
    break;
  }

  case IGNORE_TURN:
  {
    ignoreTurn();
    Bover->Stop();
    delay(200);
    movement = CONTINUE;
    break;
  }
}
}

void setup() 
{
  SerialBT.begin("Bover");
}

void loop() 
{
  if (SerialBT.available()) {
    
    Serial.print("INICIAR? ");
    bool stateStart = SerialBT.read();
      if(stateStart is true) {
        SensorsRead();
        movementLogic();
        if (DEBUG_SENSORS) printSensors();
      }
  }
}
