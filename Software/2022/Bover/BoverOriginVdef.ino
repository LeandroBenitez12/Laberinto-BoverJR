#include <PID.h>
#include <Button.h>
#include <Encoder.h>
#include <Motor.h>
#include <Ultrasonido.h>
#include "BluetoothSerial.h"

//debug
#define DEBUG 1
#define TICK_DEBUG 500
unsigned long currentTimeSensors = 0;
unsigned long currentTimeStatus = 0;
unsigned long currentTimeDebugPID = 0;



#define TICK_STOP 300
#define TICK_FORWARD 400

// declaramos pines
// motores
#define PIN_ENGINE_ENA 15
#define PIN_ENGINE_MR1 2
#define PIN_ENGINE_MR2 4
#define PIN_ENGINE_ENB 19
#define PIN_ENGINE_ML1 18
#define PIN_ENGINE_ML2 5

//ultrasonidos
#define PIN_ECCHO_RIGHT 14
#define PIN_TRIGG_RIGHT 12
#define PIN_ECCHO_LEFT 35
#define PIN_TRIGG_LEFT 25
#define PIN_ECCHO_FRONT 32
#define PIN_TRIGG_FRONT 25
float frontDistance;
float rightDistance;
float leftDistance;
#define MAX_FRONT_DISTANCE 100
#define MAX_SIDE_DISTANCE 70


// Button
#define PIN_BUZZER 23
#define PIN_BUTTON_START 13
bool button_start;

//encoders
#define PIN_ENCODER_RIGHT 13
#define PIN_ENCODER_LEFT 23
int turn_90 = 7;
int turn_180 = 14;

//veocidades motores pwm
int speedRight = 180;
int speedLeft = 180;
int averageSpeed = 180;
int speedTurn = 170;
const int PWMChannel1 = 0;
const int PWMChannel2 = 1;

//variables pid
double kp = 0.1721;
double kd = 0;
double setPoint;
float PID1;
#define TICK_PID 70

// establecemos conexion bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//instancio los objetos
Motor *engineRigh = new  Motor(PIN_ENGINE_MR1, PIN_ENGINE_MR2, PIN_ENGINE_ENA, PWMChannel1);
Motor *engineLeft = new Motor(PIN_ENGINE_ML1, PIN_ENGINE_ML2, PIN_ENGINE_ENB, PWMChannel2);

Button *start = new Button(PIN_BUTTON_START);

Encoder *encoderRight = new Encoder(PIN_ENCODER_RIGHT);
Encoder *encoderLeft = new Encoder(PIN_ENCODER_LEFT);

Ultrasonido *ultrasoundRight = new Ultrasonido(PIN_TRIGG_RIGHT, PIN_ECCHO_RIGHT);
Ultrasonido *ultrasoundLeft = new Ultrasonido(PIN_TRIGG_LEFT, PIN_ECCHO_LEFT);
Ultrasonido *ultrasoundFront = new Ultrasonido(PIN_TRIGG_FRONT, PIN_ECCHO_FRONT);

Pid *engine = new Pid(kp, kd, setPoint, TICK_PID);


// funciones de los motores
void forward()
{
  engineRigh->SetVelocidad(speedRight);
  engineLeft->SetVelocidad(speedLeft);
  engineRigh->Forward();
  engineLeft->Forward();
}

void backward()
{
  engineRigh->SetVelocidad(speedRight);
  engineLeft->SetVelocidad(speedLeft);
  engineRigh->Backward();
  engineLeft->Backward();
}

void left()
{
  engineRigh->SetVelocidad(speedRight);
  engineLeft->SetVelocidad(speedLeft);
  engineRigh->Forward();
  engineLeft->Backward();
}

void right()
{
  engineRigh->SetVelocidad(speedRight);
  engineLeft->SetVelocidad(speedLeft);
  engineRigh->Backward();
  engineLeft->Forward();
}

void stop()
{
  engineRigh->SetVelocidad(0);
  engineLeft->SetVelocidad(0);
  engineRigh->Stop();
  engineLeft->Stop();
}

void turnRight(int turn)
{
  int contRight = encoderRight->Cont();
  int contLeft = encoderLeft->Cont();
  while(contRight <= turn && contLeft <= turn)
  {
    contRight = encoderRight->Cont();
    contLeft = encoderLeft->Cont();
    engineRigh->SetVelocidad(speedTurn);
    engineRigh->Backward();
    engineLeft->SetVelocidad(speedTurn);
    engineLeft->Forward();
  }
  encoderRight->SetCont(0);
  encoderLeft->SetCont(0);
}

void turnLeft(int turn)
{
  int contRight = encoderRight->Cont();
  int contLeft = encoderLeft->Cont();
  while(contRight <= turn && contLeft <= turn)
  {
    contRight = encoderRight->Cont();
    contLeft = encoderLeft->Cont();
    engineRigh->SetVelocidad(speedTurn);
    engineRigh->Forward();
    engineLeft->SetVelocidad(speedTurn);
    engineLeft->Backward();
  }
  encoderRight->SetCont(0);
  encoderLeft->SetCont(0);
}

void continuePostTurn()
{
  int tickContinue = 10;
  int contRight = encoderRight->Cont();
  int contLeft = encoderLeft->Cont();
  while (contRight <= tickContinue && contLeft <= tickContinue)
  {
    void forward();
  }
  encoderRight->SetCont(0);
  encoderLeft->SetCont(0);
}

void continueAntTurn()
{
  int tickContinue = 10;
  int contRight = encoderRight->Cont();
  int contLeft = encoderLeft->Cont();
  while (contRight <= tickContinue && contLeft <= tickContinue)
  {
    void forward();
  }
  encoderRight->SetCont(0);
  encoderLeft->SetCont(0);
}

// casos de la maquina de estado
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

//logica de movimiento
void robotMovements()
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
    turnRight(turn_90);
    movement = POST_TURN;
    break;
  }

  case LEFT_TURN:
  {
    turnLeft(turn_90);
    movement = POST_TURN;
    break;
  }

  case FULL_TURN:
  {
    turnLeft(turn_180);
    movement = POST_TURN;
    break;
  }

  case POST_TURN:
  {
    continuePostTurn();
    break;
  }

  case ANT_TURN:
  {
    continueAntTurn();
    movement = LEFT_TURN;
    break;
  }
  }
}

void printStatus()
{
  if (millis() > currentTimeStatus + TICK_DEBUG)
  {
    currentTimeStatus = millis();
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

void printSensors()
{
  if (millis() > currentTimeSensors + TICK_DEBUG)
  {
    currentTimeSensors = millis();
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

void setup() 
{
  SerialBT.begin("BoverOrigin");
  Serial.begin(9600);
}

void loop() 
{
  frontDistance = ultrasoundRight->SensorRead();
  rightDistance = ultrasoundLeft->SensorRead();
  leftDistance = ultrasoundFront->SensorRead();
  float input = rightDistance - leftDistance;
  PID1 = engine->ComputePid(input);
  robotMovements();

  if(DEBUG)
  {
    printStatus();
    printSensors();
    printPID();
  }

} 
