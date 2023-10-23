#include <PID.h>
#include <EngineController.h>
#include <DistanceSensors.h>
#include <Button.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// debug
#define TICK_DEBUG_ALL 1000
#define DEBUG_BUTTON 1
#define DEBUG_STATUS 1
#define DEBUG_SENSORS 1
#define DEBUG_PID 1
unsigned long currentTimePID = 0;
unsigned long currentTimeDebugAll = 0;
unsigned long currentTimeMenu = 0;

// Motores
#define PIN_RIGHT_ENGINE_IN1 26
#define PIN_RIGHT_ENGINE_IN2 27
#define PIN_LEFT_ENGINE_IN1 18
#define PIN_LEFT_ENGINE_IN2 19
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4

// Sharps
#define PIN_SHARP_RIGHT 25
#define PIN_SHARP_LEFT 35
#define PIN_SHARP_FRONT 33
float rightDistance;
float leftDistance;
float frontDistance;
#define PARED_ENFRENTE 10
#define PARED_COSTADO_PASILLO 22
#define NO_HAY_PARED 25

// veocidades motores pwm
#define VELOCIDAD_GIROS_90 247
#define GIROS_90_DELAY 180
#define GIROS_180_DELAY 400
#define ENTRAR_EN_PASILLO 200
#define DELAY_TOMAR_DECISION 100
#define DELAY_ANTI_INERCIA 100
#define TICK_ANT_TURN 120

int speedRightPID = 210;
int speedLeftPID = 190;
int averageSpeedRight = 210;
int averageSpeedLeft = 190;

// variables pid
double kp = 1;
double kd = 0.14;
double ki = 0.14;
double setPoint;
float gananciaPID;
double TICK_PID = 20;

// Boton
#define PIN_BUTTON_START 32
bool start = 0;
bool wall;

IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *Bover = new EngineController(rightEngine, leftEngine);
Isensor *SharpRigh = new Sharp_GP2Y0A21(PIN_SHARP_RIGHT);
Isensor *SharpLeft = new Sharp_GP2Y0A21(PIN_SHARP_LEFT);
Isensor *SharpFront = new Sharp_GP2Y0A21(PIN_SHARP_FRONT);
Pid *PID = new Pid(kp, kd, ki, setPoint, TICK_PID);

Button *buttonStart1 = new Button(PIN_BUTTON_START);
void SensorsRead()
{

  frontDistance = SharpFront->SensorRead();
  rightDistance = SharpRigh->SensorRead();
  leftDistance = SharpLeft->SensorRead();
}

bool stateStartButton;

void printButton()
{
  SerialBT.print("Button Start: ");
  SerialBT.println(stateStartButton);
}
void printPID()
{
  if (millis() > currentTimePID + TICK_DEBUG_ALL)
  {
    currentTimePID = millis();
    SerialBT.println("");
    SerialBT.print("Ganancia PID: ");
    SerialBT.println(gananciaPID);
    SerialBT.print("speedRight: ");
    SerialBT.print(speedRightPID);
    SerialBT.print(" || speedLeft: ");
    SerialBT.println(speedLeftPID);
    SerialBT.println("");
  }
}

void printSensors()
{
  SerialBT.print("LeftDistance: ");
  SerialBT.println(leftDistance);
  SerialBT.print("frontDistance: ");
  SerialBT.println(frontDistance);
  SerialBT.print(" || rightDistance: ");
  SerialBT.println(rightDistance);
}

void turnRight()
{
  Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_90_DELAY);
}

void turnLeft()
{
  Bover->Left(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_90_DELAY);
}

void fullTurn()
{
  Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_180_DELAY);
}

void postTurn()
{
  Bover->Forward(averageSpeedRight, averageSpeedLeft);
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
};
int movement = STANDBY;

void movementLogic()
{
  switch (movement)
  {
  case STANDBY:
  {
    bool stateStartButton = buttonStart1->GetIsPress();
    Bover->Stop();
    if (stateStartButton)
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
    if (DEBUG_PID)
      printPID();

    speedRightPID = (averageSpeedRight + (gananciaPID));
    speedLeftPID = (averageSpeedLeft - (gananciaPID));

    Bover->Forward(speedRightPID, speedLeftPID);

    if (frontDistance <= PARED_ENFRENTE)
      movement = STOP;
    break;
  }
  case STOP:
  {
    Bover->Stop();
    delay(DELAY_TOMAR_DECISION);

    if (frontDistance <= PARED_ENFRENTE && rightDistance > NO_HAY_PARED && leftDistance <= PARED_COSTADO_PASILLO)
      movement = RIGHT_TURN;
    if (frontDistance <= PARED_ENFRENTE && rightDistance <= PARED_COSTADO_PASILLO && leftDistance > NO_HAY_PARED)
      movement = LEFT_TURN;
    if (frontDistance <= PARED_ENFRENTE && rightDistance > NO_HAY_PARED && leftDistance > NO_HAY_PARED)
      movement = RIGHT_TURN;
    if (frontDistance <= PARED_ENFRENTE && rightDistance <= PARED_COSTADO_PASILLO && leftDistance <= PARED_COSTADO_PASILLO)
      movement = FULL_TURN;

    else if (frontDistance > PARED_ENFRENTE)
      movement = CONTINUE;
    break;
  }

  case RIGHT_TURN:
  {
    turnRight();
    Bover->Stop();
    delay(DELAY_ANTI_INERCIA);
    movement = POST_TURN;
    break;
  }

  case LEFT_TURN:
  {
    turnLeft();
    Bover->Stop();
    delay(DELAY_ANTI_INERCIA);
    movement = POST_TURN;
    break;
  }

  case FULL_TURN:
  {
    fullTurn();
    Bover->Stop();
    delay(DELAY_ANTI_INERCIA);
    movement = POST_TURN;
    break;
  }

  case POST_TURN:
  {
    postTurn();
    Bover->Stop();
    delay(DELAY_ANTI_INERCIA);
    movement = CONTINUE;
    break;
  }
}
}

void printStatus()
{
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
  }
  SerialBT.print("State: ");
  SerialBT.println(state);
}

void printAll()
{
  if (millis() > currentTimeDebugAll + TICK_DEBUG_ALL)
  {
    currentTimeDebugAll = millis();
    if (DEBUG_BUTTON)
      printButton();
      SerialBT.println("");
    if (DEBUG_SENSORS)
      printSensors();
      SerialBT.println("");
    if (DEBUG_STATUS)
      printStatus();
      SerialBT.println("");
  }
}
void setup()
{
  SerialBT.begin("Bover");
}

void loop()
{
  SensorsRead();
  movementLogic();
  printAll();
}