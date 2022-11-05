#include <PID.h>
#include <Engine.h>
#include <Button.h>
#include <QRE1113.h>
#include <SharpLab.h>
#include "BluetoothSerial.h"


//debug
#define DEBUG 1
#define TICK_DEBUG 400
unsigned long currentTimeDebug = 0;
unsigned long currentTimeDebugSensors = 0;
unsigned long currentTimeDebugPID = 0;

// declaramos pines
// motores
#define PIN_ENGINE_MR1 23
#define PIN_ENGINE_MR2 22
#define PIN_ENGINE_ML1 19
#define PIN_ENGINE_ML2 18

// Button
#define PIN_BUTTON_START 39
bool button_start;

//Sensor final
#define PIN_END_SENSOR 32

//Sharps
#define PIN_SHARP_RIGHT 34
#define PIN_SHARP_LEFT 27
#define PIN_SHARP_FRONT 35
float frontDistance;
float rightDistance;
float leftDistance;
#define MAX_FRONT_DISTANCE 20
#define MAX_SIDE_DISTANCE 12

//encoder 
int interruptPinRight = 26;
int interruptPinLeft = 21;
volatile long rightCont = 0;
volatile long leftCont = 0;
#define TICK_TURN_RIGHT_90 17
#define TICK_TURN_LEFT_90 120
#define TICK_TURN_180 34
#define TICK_POST_TURN 37
#define TICK_ANT_TURN 30
#define TICK_IGNORE_TURN 40

void rightEncRead()
{
    rightCont++;
}

void leftEncRead()
{
    leftCont++;
}

//veocidades motores pwm
int speedRight = 80;
int speedLeft = 80;
int averageSpeed = 80;
int speedTurn = 80;

//variables pid
double kp = 0.175;
double kd = 0;
double setPoint;
float PID1;
double TICK_PID = 50;


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//instancio los objetos
Engine *engineRigh = new  Engine(PIN_ENGINE_MR1, PIN_ENGINE_MR2);
Engine *engineLeft = new Engine(PIN_ENGINE_ML1, PIN_ENGINE_ML2);

Button *start = new Button(PIN_BUTTON_START);

QRE1113 *endSensor = new QRE1113(PIN_END_SENSOR);

Sharp *SharpFront = new Sharp(PIN_SHARP_FRONT);
Sharp *SharpRigh = new Sharp(PIN_SHARP_RIGHT);
Sharp *SharpLeft = new Sharp(PIN_SHARP_LEFT);

Pid *engine = new Pid(kp, kd, setPoint, TICK_PID);

// funciones de los motores
void forward()
{
  engineRigh->SetSpeed(speedRight);
  engineLeft->SetSpeed(speedLeft);
  engineRigh->Forward();
  engineLeft->Forward();
}

void backward()
{
  engineRigh->SetSpeed(speedRight);
  engineLeft->SetSpeed(speedLeft);
  engineRigh->Backward();
  engineLeft->Backward();
}

void left()
{
  engineRigh->SetSpeed(speedRight);
  engineLeft->SetSpeed(speedLeft);
  engineRigh->Forward();
  engineLeft->Backward();
}

void right()
{
  engineRigh->SetSpeed(speedRight);
  engineLeft->SetSpeed(speedLeft);
  engineRigh->Backward();
  engineLeft->Forward();
}

void stop()
{
  engineRigh->SetSpeed(0);
  engineLeft->SetSpeed(0);
  engineRigh->Stop();
  engineLeft->Stop();
}

void turnRight()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < TICK_TURN_RIGHT_90 || leftCont < TICK_TURN_RIGHT_90)
  {
    engineRigh->SetSpeed(speedTurn);
    engineLeft->SetSpeed(speedTurn);
    engineRigh->Backward();
    engineLeft->Forward();
  }
}

void turnLeft()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < TICK_TURN_LEFT_90 || leftCont < TICK_TURN_LEFT_90)
  {
    engineRigh->SetSpeed(speedTurn);
    engineLeft->SetSpeed(speedTurn);
    engineRigh->Forward();
    engineLeft->Backward();
  }
}

void fulTurn()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < TICK_TURN_180 || leftCont < TICK_TURN_180)
  {
    engineRigh->SetSpeed(speedTurn);
    engineLeft->SetSpeed(speedTurn);
    engineRigh->Forward();
    engineLeft->Backward();
  }
}

void antTurn()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < TICK_ANT_TURN || leftCont < TICK_ANT_TURN)
  {
    engineRigh->SetSpeed(averageSpeed);
    engineLeft->SetSpeed(averageSpeed);
    engineRigh->Forward();
    engineLeft->Forward();
  }
}

void posTurn()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < TICK_POST_TURN || leftCont < TICK_POST_TURN)
  {
    engineRigh->SetSpeed(averageSpeed);
    engineLeft->SetSpeed(averageSpeed);
    engineRigh->Forward();
    engineLeft->Forward();
  }
}

void ignoreTurn()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < TICK_IGNORE_TURN || leftCont < TICK_IGNORE_TURN)
  {
    engineRigh->SetSpeed(averageSpeed);
    engineLeft->SetSpeed(averageSpeed);
    engineRigh->Forward();
    engineLeft->Forward();
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
    button_start = start->GetIsPress();
    if (button_start)
    {
      movement = CONTINUE;
    }

    stop();
    break;
  }

  case CONTINUE:
  {
    if (PID1)
    {
      if (PID1 > 30) PID1 = 30;
      if (PID1 < -30) PID1 = -30;

      speedRight = (averageSpeed - (PID1));
      speedLeft = (averageSpeed + (PID1));
    }
    forward();
    if (frontDistance < MAX_FRONT_DISTANCE) movement = STOP;
    if (leftDistance > MAX_SIDE_DISTANCE) movement = ANT_TURN;
    else if (rightDistance > MAX_SIDE_DISTANCE) movement = IGNORE_TURN;
    break;
  }

  case STOP:
  {
    stop();
    delay(100);
    if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = LEFT_TURN;
    if (rightDistance >= MAX_SIDE_DISTANCE && leftDistance < MAX_SIDE_DISTANCE) movement = RIGHT_TURN;
    if (rightDistance >= MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = LEFT_TURN;
    if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance < MAX_SIDE_DISTANCE) movement = FULL_TURN;
    break;
  }

  case RIGHT_TURN:
  {
    turnRight();
    movement = POST_TURN;
    break;
  }

  case LEFT_TURN:
  {
    turnLeft();
    movement = POST_TURN;
    break;
  }

  case FULL_TURN:
  {
    fulTurn();
    movement = POST_TURN;
    break;
  }

  case POST_TURN:
  {
    posTurn();
    movement = CONTINUE;
    break;
  }

  case ANT_TURN:
  {
    antTurn();
    movement = LEFT_TURN;
    break;
  }

  case IGNORE_TURN:
  {
    ignoreTurn();
    movement = CONTINUE;
    break;
  }
}
}


void printSensors()
{
  if (millis() > currentTimeDebugSensors + TICK_DEBUG)
  {
    currentTimeDebugSensors = millis();
    SerialBT.print("frontDistance: ");
    SerialBT.print(frontDistance);
    SerialBT.print(" || rightDistance: ");
    SerialBT.print(rightDistance);
    SerialBT.print(" || leftDistance: ");
    SerialBT.println(leftDistance);
  }
}

void printPID()
{
  if (millis() > currentTimeDebugPID + TICK_DEBUG)
  {
    currentTimeDebugPID = millis();
    SerialBT.print("PID: ");
    SerialBT.println(PID1);
    SerialBT.print("speedRight: ");
    SerialBT.print(speedRight);
    SerialBT.print(" || speedLeft: ");
    SerialBT.println(speedLeft);
  }
}

void printStatus()
{
  if (millis() > currentTimeDebug + TICK_DEBUG)
  {
    currentTimeDebug = millis();
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
    }
    SerialBT.print("State: ");
    SerialBT.println(state);
  }
  }

void setup() 
{
  SerialBT.begin("Bover");
  attachInterrupt(digitalPinToInterrupt(interruptPinRight),rightEncRead, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPinLeft),leftEncRead, RISING);
}
 

void loop() 
{
  frontDistance = SharpFront->SharpDist();
  rightDistance = SharpRigh->SharpDist();
  leftDistance = SharpLeft->SharpDist();
  float input = rightDistance - leftDistance;
  PID1 = engine->ComputePid(input);
  movementLogic();
  if(DEBUG)
  {
    printStatus();
    printSensors();
    printPID();
  } 
}
