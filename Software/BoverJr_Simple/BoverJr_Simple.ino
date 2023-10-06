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
#define TICK_DEBUG_ALL 1000
#define DEBUG_STATUS 1
#define DEBUG_SENSORS 1
#define DEBUG_PID 1
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
#define MAX_FRONT_DISTANCE 12
#define MAX_SIDE_DISTANCE 23
#define MAX_DISTANCE_ANT_TURN 27

//veocidades motores pwm
#define VELOCIDAD_GIROS_90 160
#define GIROS_90_DELAY 200
#define GIROS_180_DELAY 400
#define ENTRAR_EN_PASILLO 200
#define STOPS_PARA_PENSAR 100
int speedRight = 80;
int speedLeft = 80;
int speedRightPID;
int speedLeftPID;
int averageSpeedRight = 120;
int averageSpeedLeft = 110;

//variables pid
double kp = 1.2;
double kd = 0.3;
double setPoint;
float gananciaPID;
double TICK_PID = 50;

//Boton
#define PIN_BUTTON_START 32
bool start = 0;
bool wall;

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

bool stateStartButton;
void printButton(){
  SerialBT.print("Button Start: ");
  SerialBT.println(stateStartButton);
  SerialBT.println("______");
}
void printPID()
{
  if (millis() > currentTimePID + TICK_DEBUG_ALL)
  {
    currentTimePID = millis();
    SerialBT.println("______");
    SerialBT.print("Ganancia PID: ");
    SerialBT.println(gananciaPID);
    SerialBT.print("speedRight: ");
    SerialBT.print(speedRightPID);
    SerialBT.print(" || speedLeft: ");
    SerialBT.println(speedLeftPID);
    SerialBT.println("______");
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
  Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_90_DELAY);
}

void turnLeft(){
  Bover->Left(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_90_DELAY);
}

void postTurn(){
  Bover->Forward(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(ENTRAR_EN_PASILLO);
}
void antTurn(){
  Bover->Forward(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(5);
}

void fullTurn(){
  Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_180_DELAY);
} 

void ignoreTurn(){
  Bover->Forward(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(ENTRAR_EN_PASILLO);
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
    stateStartButton = buttonStart->GetIsPress();
    Bover->Stop();
    SerialBT.print("Que pared Seguir( '0' = IZQ/ '1' = DER):   ");
    wall = SerialBT.read();
    SerialBT.print("INICIAR? ");
    bool stateStartBluetooh = SerialBT.read();
    if (stateStartBluetooh || stateStartButton) movement = CONTINUE;
    break;
  }

  case CONTINUE:
  {
    float input = rightDistance - leftDistance;
    gananciaPID = PID->ComputePid(input);
    if(DEBUG_PID) printPID();
    speedRightPID = (averageSpeedRight - (gananciaPID));
    speedLeftPID = (averageSpeedLeft + (gananciaPID));
    Bover->Forward(speedRightPID, speedLeftPID);
    
    if (frontDistance < MAX_FRONT_DISTANCE) movement = STOP;
    if(wall == true){
      if (rightDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN && leftDistance < MAX_SIDE_DISTANCE ) movement = ANT_TURN;
      if (leftDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN && rightDistance < MAX_SIDE_DISTANCE) movement = IGNORE_TURN;
    }
    else if(wall == false){
      if (rightDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN && leftDistance < MAX_SIDE_DISTANCE ) movement = IGNORE_TURN;
      if (leftDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN && rightDistance < MAX_SIDE_DISTANCE) movement = ANT_TURN;
    }
    break;
  }

  case STOP:
  {
    Bover->Stop();
    delay(STOPS_PARA_PENSAR);
    if (frontDistance < MAX_FRONT_DISTANCE){
      if (rightDistance > MAX_SIDE_DISTANCE && leftDistance <= MAX_SIDE_DISTANCE) movement = RIGHT_TURN;
      if (rightDistance > MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = RIGHT_TURN;
      if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = LEFT_TURN;
      if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance <= MAX_SIDE_DISTANCE) movement = FULL_TURN;
    }
    else movement = CONTINUE;
    break;
  }

  case RIGHT_TURN:
  {
    turnRight();
    Bover->Stop();
    delay(STOPS_PARA_PENSAR);
    movement = POST_TURN;
    break;
  }

  case LEFT_TURN:
  {
    turnLeft();
    Bover->Stop();
    delay(STOPS_PARA_PENSAR);
    movement = POST_TURN;
    break;
  }

  case FULL_TURN:
  {
    fullTurn();
    Bover->Stop();
    delay(STOPS_PARA_PENSAR);
    movement = POST_TURN;
    break;
  }

  case POST_TURN:
  {
    postTurn();
    Bover->Stop();
    delay(STOPS_PARA_PENSAR);
    movement = CONTINUE;
    break;
  }

  case ANT_TURN:
  {
    antTurn();
    Bover->Stop();
    delay(STOPS_PARA_PENSAR);
    movement = RIGHT_TURN;
    break;
  }

  case IGNORE_TURN:
  {
    ignoreTurn();
    Bover->Stop();
    delay(STOPS_PARA_PENSAR);
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
    if(DEBUG_BUTTON) printButton();
    if (DEBUG_SENSORS) printSensors();
    SerialBT.println("______");
    if (DEBUG_STATUS) printStatus();
    SerialBT.println("_____________________");
  }
}
void setup() 
{
  SerialBT.begin("Bover");
  pinMode(PIN_BUTTON_START, INPUT_PULLUP);
}

void loop() 
{    
  SensorsRead();
  movementLogic();
  printAll();
}
