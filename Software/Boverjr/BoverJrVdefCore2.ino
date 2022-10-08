#include <PID.h>
#include <Engine.h>
#include <Button.h>
#include <QRE1113.h>
#include <SharpLab.h>
#include "BluetoothSerial.h"

TaskHandle_t Task1;
TaskHandle_t Task2;

// debug
#define DEBUG 1
#define TICK_DEBUG 400
unsigned long currentTimeDebug = 0;
unsigned long currentTimeDebugSensors = 0;
unsigned long currentTimeDebugPID = 0;

// declaramos pines
// motores
#define PIN_ENGINE_MR1 23
#define PIN_ENGINE_MR2 22
#define PIN_ENGINE_ML1 18
#define PIN_ENGINE_ML2 19

// Button
#define PIN_BUTTON_START 39
bool button_start;

// Sensor final
#define PIN_END_SENSOR 32

// Sharps
#define PIN_SHARP_RIGHT 27
#define PIN_SHARP_LEFT 34
#define PIN_SHARP_FRONT 35
float frontDistance;
float rightDistance;
float leftDistance;
#define MAX_FRONT_DISTANCE 20
#define MAX_SIDE_DISTANCE 15

// veocidades motores pwm
int speedRight = 100;
int speedLeft = 100;
int averageSpeed = 100;
int speedTurn = 120;

#define TICK_STOP 1000
#define TICK_TURN 360
#define TICK_FORWARD 200

// variables pid
double kp = 0.1721;
double kd = 0;
double setPoint;
float PID1;
#define TICK_PID 70

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// instancio los objetos
Engine *engineRigh = new Engine(PIN_ENGINE_MR1, PIN_ENGINE_MR2);
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
    }
    forward();
    if (frontDistance < MAX_FRONT_DISTANCE)
      movement = STOP;
    // if (leftDistance > MAX_SIDE_DISTANCE) movement = ANT_TURN;
    break;
  }

  case STOP:
  {
    stop();
    delay(TICK_STOP);
    if (rightDistance < MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE)
      movement = LEFT_TURN;
    if (rightDistance > MAX_SIDE_DISTANCE && leftDistance < MAX_SIDE_DISTANCE)
      movement = RIGHT_TURN;
    if (rightDistance > MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE)
      movement = LEFT_TURN;
    if (rightDistance < MAX_SIDE_DISTANCE && leftDistance < MAX_SIDE_DISTANCE)
      movement = FULL_TURN;
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
    case STANDBY:
      state = "STANDBY";
      break;
    case CONTINUE:
      state = "CONTINUE";
      break;
    case STOP:
      state = state = "STOP";
      break;
    case RIGHT_TURN:
      state = "RIGHT TURN";
      break;
    case LEFT_TURN:
      state = "LEFT TURN";
      break;
    case FULL_TURN:
      state = "FULL TURN";
      break;
    case POST_TURN:
      state = "POST TURN";
      break;
    case ANT_TURN:
      state = "ANT TURN";
      break;
    }
    SerialBT.print("State: ");
    SerialBT.println(state);
  }
}

void sensors(void *parameter)
{
  for (;;)
  {
    frontDistance = SharpFront->SharpDist();
    rightDistance = SharpRigh->SharpDist();
    leftDistance = SharpLeft->SharpDist();
    float input = rightDistance - leftDistance;
    PID1 = engine->ComputePid(input);
  }
}

void movement(void *parameter) {
    for(;;) {
        movementLogic();
    }
}

void setup()
{
  //funcion para crear la nueva tarea para que se ejecute en el nucleo 0
  xTaskCreatePinnedToCore(
    sensors, // funcion 
    "tasksensors", //nombre de la funcion
    1000, //tamaño de la pila
    NULL, //parametro a pasarle a la tarea
    1, // setea la prioridad de la tarea
    &Task1, //nombre de la variable 
    0); //en el nucleo en el que se ejecuta la tarea
    

    //funcion para crear la nueva tarea para que se ejecute en el nucleo 1
  xTaskCreatePinnedToCore(
    movement, // funcion 
    "taskMovement", //nombre de la funcion
    1000, //tamaño de la pila
    NULL, //parametro a pasarle a la tarea
    1, // setea la prioridad de la tarea
    &Task2, //nombre de la variable 
    1); //en el nucleo en el que se ejecuta la tarea
  SerialBT.begin("Bover");
  Serial.begin(9600);
}

void loop()
{
  if (DEBUG)
  {
    printStatus();
    printSensors();
    printPID();
  }
}