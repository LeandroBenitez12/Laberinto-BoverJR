#include <PID.h>
#include <Engine.h>
#include <Button.h>
#include <QRE1113.h>
#include <SharpLab.h>
#include "BluetoothSerial.h"


//debug
#define DEBUG 1
#define TICK_DEBUG 500
unsigned long currentTimeDebug = 0;

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
#define PIN_SHARP_RIGH
#define PIN_SHARP_LEFT
#define PIN_SHARP_FRONT
float frontDistance;
float rightDistance;
float leftDistance;
#define MAX_FRONT_DISTANCE 10
#define MAX_SIDE_DISTANCE 7

//veocidades motores pwm
int speedRight = 150;
int speedLeft = 150;
int averageSpeed = 150;
int speedTurn = 130;

#define TICK_STOP 500
#define TICK_TURN 200
#define TICK_FORWARD 300

//variables pid
double kp = 0.1721;
double kd = 0;
double setPoint;
float PID1;
double TICK_PID =  70;


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//instancio los objetos
Engine *engineRigh = new  Engine(PIN_ENGINE_MR1, PIN_ENGINE_MR2);
Engine *engineLeft = new Engine(PIN_ENGINE_ML1, PIN_ENGINE_ML2);

Button *start = new Button(PIN_BUTTON_START);

QRE1113 *endSensor = new QRE1113(PIN_END_SENSOR);

Sharp *SharpFront = new Sharp(1);
Sharp *SharpRigh = new Sharp(1);
Sharp *SharpLeft = new Sharp(1);

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

enum movement
{
  STANDBY,
  CONTINUE,
  STOP,
  RIGHT_TURN,
  LEFT_TURN,
  FULL_TURN,
  POST_TURN,
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
        if (PID1 > 30)
        PID1 = 30;
        if (PID1 < -30)
        PID1 = -30;

      speedRight = (averageSpeed - (PID1));
      speedLeft = (averageSpeed + (PID1));

      
      if (speedLeft < 160)
        speedLeft = 160;
      if (speedRight < 160)
        speedRight = 160;
    }
    forward();
    if (frontDistance < MAX_FRONT_DISTANCE) movement = STOP;
    if (leftDistance > MAX_SIDE_DISTANCE) movement = ANT_TURN;
    break;
  }

  case STOP:
  {
    stop();
    delay(TICK_STOP);

    if (rightDistance < MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = LEFT_TURN;
    if (rightDistance > MAX_SIDE_DISTANCE && leftDistance < MAX_SIDE_DISTANCE) movement = RIGHT_TURN;
    if (rightDistance > MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE) movement = LEFT_TURN;
    if (rightDistance < MAX_SIDE_DISTANCE && leftDistance < MAX_SIDE_DISTANCE) movement = FULL_TURN;
    break;
  }

  case RIGHT_TURN:
  {
    right();
    delay(TICK_TURN);
    movement = POST_TURN;
    
    break;
  }

  case LEFT_TURN:
  {
    left();
    delay(TICK_TURN);
    movement = POST_TURN;

    break;
  }

  case FULL_TURN:
  {
    right();
    delay(TICK_TURN);
    delay(TICK_TURN);
    movement = POST_TURN;

    break;
  }

  case POST_TURN:
  {
    forward();
    delay(TICK_FORWARD);
    movement = CONTINUE;
    break;
  }

  case ANT_TURN:
  {
    forward();
    delay(TICK_FORWARD);
    movement = LEFT_TURN;
    break;
  }

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
    SerialBT.print(state);
  }
  }

void setup() 
{
  SerialBT.begin("Bover");
  Serial.begin(9600);
}

void loop() 
{
  frontDistance = SharpFront->SharpDist();
  rightDistance = SharpRigh->SharpDist();
  leftDistance = SharpLeft->SharpDist();
  float input = rightDistance - leftDistance;
  PID1 = engine->ComputePid(input);
  movementLogic();
  if(DEBUG) printStatus();
}