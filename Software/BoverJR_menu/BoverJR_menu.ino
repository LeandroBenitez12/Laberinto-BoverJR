#include <PID.h>
#include <EngineController.h>
#include <DistanceSensors.h>
#include <Button.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//meniu 
bool menusalir = false;
bool wall = false;
// debug
#define TICK_DEBUG_ALL 1000
#define DEBUG_BUTTON 1
#define DEBUG_STATUS 1
#define DEBUG_SENSORS 1
#define DEBUG_PID 1
unsigned long currentTimePID = 0;
unsigned long currentTimeDebugAll = 0;
unsigned long currentTimeMenu = 0;
#define TICK_MENU 5000

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
#define PARED_ENFRENTE 7
#define PARED_COSTADO_PASILLO 22
#define NO_HAY_PARED 26

// veocidades motores pwm
#define VELOCIDAD_GIROS_90 255
int tick_giro_90 = 300;
int tick_giro_180 = 600;
#define ENTRAR_EN_PASILLO 500
#define DELAY_TOMAR_DECISION 100
#define DELAY_ANTI_INERCIA 50

#define MAX_VEL 255
int speedRightPID;
int speedLeftPID;
int averageSpeedRight = 220;
int averageSpeedLeft = 205;

// variables pid
double kp = 1.9;
double kd = 0.59;
double ki = 0.000000000000;
double setPoint;
double gananciaPID;
double TICK_PID = 1;

// Boton
#define PIN_BUTTON_START 32
bool start = 0;

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
  delay(tick_giro_90);
}

void turnLeft()
{
  Bover->Left(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(tick_giro_90);
}

void fullTurn()
{
  Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(tick_giro_180);
}

void postTurn()
{
  Bover->Forward(averageSpeedRight, averageSpeedLeft);
  delay(ENTRAR_EN_PASILLO);
}

enum menu{
  READ_MSG,
  WALL_TO_FOLLOW,
  SPEED_RIGHT,
  SPEED_LEFT,
  KP,
  KI,
  KD,
  TICK_90,
  TICK_180,
  GO_OUT
};
int menu = READ_MSG;
void funcionMenu(){
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n'); // Lee el comando recibido
    switch(menu){
      case READ_MSG:{
        if( millis() > currentTimeMenu + TICK_MENU){
          currentTimeMenu = millis();
          SerialBT.println("Ingrese comando");
          SerialBT.println("wtf");
          SerialBT.println("sr");
          SerialBT.println("sl");
          SerialBT.println("kp");
          SerialBT.println("ki");
          SerialBT.println("kd");
          SerialBT.println("90");
          SerialBT.println("180");
          SerialBT.println("d");
        }
        if (command.startsWith("wtf")) menu = WALL_TO_FOLLOW;
        if (command.startsWith("sr")) menu = SPEED_RIGHT;
        if (command.startsWith("sl")) menu = SPEED_LEFT;
        if (command.startsWith("kp")) menu = KP;
        if (command.startsWith("ki")) menu = KI;
        if (command.startsWith("kd")) menu = KD;
        if (command.startsWith("90")) menu = TICK_90;
        if (command.startsWith("180")) menu = TICK_180;
        if (command.startsWith("d")) menu = GO_OUT;
      }
      case WALL_TO_FOLLOW:{
        String command = SerialBT.readStringUntil('\n'); // Lee el comando recibido
        if (command == "1") {
          wall = true; // Cambia el valor a true
        } else if (command == "0") {
          wall = false; // Cambia el valor a false
        }
        if(command == "s") menu = READ_MSG;
        break;
      }
      case SPEED_RIGHT:{
        averageSpeedRight = command.substring(2).toInt();
        String command = SerialBT.readStringUntil('\n'); // Lee el comando recibido
        if(command == "s") menu = READ_MSG;
        break;
      }
      case SPEED_LEFT:{
        averageSpeedLeft = command.substring(2).toInt();
        String command = SerialBT.readStringUntil('\n'); // Lee el comando recibido
        if(command == "s") menu = READ_MSG;
        break;
      }
      case KP:{
        kp = command.substring(2).toDouble();
        String command = SerialBT.readStringUntil('\n'); // Lee el comando recibido
        if(command == "s") menu = READ_MSG;
        break;
      }
      case KI:{
        ki = command.substring(2).toDouble();
        String command = SerialBT.readStringUntil('\n'); // Lee el comando recibido
        if(command == "s") menu = READ_MSG;
        break;
      }
      case KD:{
        kd = command.substring(2).toDouble();
        String command = SerialBT.readStringUntil('\n'); // Lee el comando recibido
        if(command == "s") menu = READ_MSG;
        break;
      }
      case TICK_90:{
        tick_giro_90 = command.substring(2).toInt();
        String command = SerialBT.readStringUntil('\n'); // Lee el comando recibido
        if(command == "s") menu = READ_MSG;
        break;
      }
      case TICK_180:{
        tick_giro_180 = command.substring(2).toInt();
        String command = SerialBT.readStringUntil('\n'); // Lee el comando recibido
        if(command == "s") menu = READ_MSG;
        break;
      }
      case GO_OUT:{
        menusalir = true;
        break;
      }
    }
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
        delay(2000);
        movement = CONTINUE;
      }
    }
    break;
  case CONTINUE:
  {
    float input = rightDistance - leftDistance;
    gananciaPID = PID->ComputePid(input);
    if (DEBUG_PID)
      printPID();

    speedRightPID = (averageSpeedRight - (gananciaPID));
    speedLeftPID = (averageSpeedLeft + (gananciaPID));
    if(speedLeftPID >= MAX_VEL)  speedLeftPID = MAX_VEL;
    if(speedRightPID >= MAX_VEL) speedRightPID = MAX_VEL;

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

enum robot{
  MENU,
  MOVIMIENTOS
};
int robot = MENU;

void robotito(){
  switch(robot){
    case MENU:{
      if(menusalir == false){
        funcionMenu();
      }
      else if(menusalir == true){
        robot = MOVIMIENTOS;
      }
      break;
    }
    case MOVIMIENTOS:{
      if(menusalir == true) movementLogic();
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
  robotito();
  if(menusalir == true) printAll();
}