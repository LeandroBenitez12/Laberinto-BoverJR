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
<<<<<<< HEAD
#define DEBUG_BUTTON 0
#define DEBUG_STATUS 0
#define DEBUG_SENSORS 0
#define DEBUG_PID 0
#define TICK_MENU 5000
=======
#define DEBUG_BUTTON 1
#define DEBUG_STATUS 1
#define DEBUG_SENSORS 1
#define DEBUG_PID 1
>>>>>>> d0b76f3ee8112b5f2127c17d0ac5584b443cd0dd
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
#define MAX_FRONT_DISTANCE 10
#define MAX_SIDE_DISTANCE 20
#define MAX_SIDE_DISTANCE_T 28
#define MAX_DISTANCE_ANT_TURN 26

// veocidades motores pwm
#define VELOCIDAD_GIROS_90 247
#define GIROS_90_DELAY 180
#define GIROS_180_DELAY 400
#define ENTRAR_EN_PASILLO 200
#define STOPS_PARA_PENSAR 500
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
  SerialBT.println("__");
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

void postTurn()
{
  Bover->Forward(210, 190);
  delay(ENTRAR_EN_PASILLO);
}
void antTurn()
{
  Bover->Forward(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(TICK_ANT_TURN);
}

void fullTurn()
{
  Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_180_DELAY);
}

void ignoreTurn()
{
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
    bool stateStartButton = buttonStart1->GetIsPress();
    Bover->Stop();
    // SerialBT.print("Seguir(0 = IZQ || 1 = DER):   ");
    // SerialBT.println(wall);
    // wall = SerialBT.read();
    // SerialBT.print("INICIAR: ");
    // bool stateStartBluetooh = SerialBT.read();
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

    if (frontDistance < MAX_FRONT_DISTANCE)
      movement = STOP;
    if (rightDistance > MAX_SIDE_DISTANCE)
    {
      Bover->Forward(averageSpeedRight, averageSpeedLeft);
      delay(TICK_ANT_TURN);
      movement = RIGHT_TURN;
    }
    /*if (rightDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN && leftDistance < MAX_SIDE_DISTANCE ) movement = RIGHT_TURN;
    if (leftDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN && rightDistance < MAX_SIDE_DISTANCE) movement = IGNORE_TURN;

    else if(wall == false){
      if (rightDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN && leftDistance < MAX_SIDE_DISTANCE ) movement = IGNORE_TURN;
      if (leftDistance > MAX_SIDE_DISTANCE && frontDistance > MAX_DISTANCE_ANT_TURN && rightDistance < MAX_SIDE_DISTANCE) movement = ANT_TURN;
    }
    */
    break;
  }

  case STOP:
  {
    Bover->Stop();
    delay(STOPS_PARA_PENSAR);
    if (frontDistance < MAX_FRONT_DISTANCE)
    {
      if (rightDistance > MAX_SIDE_DISTANCE && leftDistance <= MAX_SIDE_DISTANCE)
        movement = RIGHT_TURN;
      if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance > MAX_SIDE_DISTANCE)
        movement = LEFT_TURN;

      if (rightDistance > MAX_SIDE_DISTANCE_T && leftDistance > MAX_SIDE_DISTANCE_T)
        movement = RIGHT_TURN;
      else
      {
        Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
        delay(60);
        movement = CONTINUE;
      }
      /*{

        if(wall == true) movement = LEFT_TURN;
        else if (wall == false) movement = RIGHT_TURN;
      }
      if (rightDistance <= MAX_SIDE_DISTANCE && leftDistance <= MAX_SIDE_DISTANCE) movement = FULL_TURN;
      */
    }
    else
      movement = CONTINUE;
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
  case ANT_TURN:
    state = "ANT TURN";
    break;
  case IGNORE_TURN:
    state = "IGNORE_TURN";
    break;
  }
  SerialBT.print("State: ");
  SerialBT.println(state);
}

enum menu
{
  ELECCION,
  KP,
  KI,
  KD,
  VEL_DERECHA,
  VEL_IZQUIERDA,
  DEBUGEAR_TODO,
  SALIR
};
int mensaje = MENU_OPCIONES;
/*
void menuConfig(){
  char OpcionSeleccionada = SerialBT.read();
  switch (OpcionSeleccionada)
  {
  case ELECCION:
  {
    opcionSeleccionada = SerialBT.read();
    if (opcionSeleccionada == "A")
      menu = KP;
    if (opcionSeleccionada == "B")
      menu = KI;
    if (opcionSeleccionada == "C")
      menu = KD;
    if (opcionSeleccionada == "D")
      menu = VEL_DERECHA;
    if (opcionSeleccionada == "E")
      menu = VEL_IZQUIERDA;
    if (opcionSeleccionada == "F")
      menu = DEBUGEAR_TODO;
    if (opcionSeleccionada == "G")
      menu = SALIR;
  }
  case KP:
  {
    int dato = SerialBT.read();
    kp = dato;
    menu = SALIR;
  }
  break;
  case KI:
  {
    int dato = SerialBT.read();
    ki = dato;
    menu = SALIR;
  }
  break;
  case KD:
  {
    int dato = SerialBT.read();
    kd = dato;
    menu = SALIR;
  }
  break;
  case VEL_DERECHA:
  {
    int dato = SerialBT.read();
    speedRightPID = dato;
    menu = SALIR;
  }
  break;
  case VEL_IZQUIERDA:
  {
    int dato = SerialBT.read();
    speedLeftPID = dato;
    menu = SALIR;
  }
  break;
  case DEBUGEAR_TODO:
  {
    /*bool dato = SerialBT.read();
    dato = DEBUG;
    DEBUG_PID = dato;
    DEBUG_SENSORS = dato;
    DEBUG_STATUS = dato;
    */
    menu = SALIR;
  }
  break;
  case SALIR:
    SerialBT.println("SALIR");
    break;
  }
}

void printAll()
{
  if (millis() > currentTimeDebugAll + TICK_DEBUG_ALL)
  {
    currentTimeDebugAll = millis();
    if (DEBUG_BUTTON)
      printButton();
    if (DEBUG_SENSORS)
      printSensors();
    SerialBT.println("__");
    if (DEBUG_STATUS)
      printStatus();
    SerialBT.println("_______");
  }
}
void setup()
{
  SerialBT.begin("Bover");

  // pinMode(PIN_BUTTON_START, INPUT_PULLUP);
}

void loop()
{
  if (SerialBT.available())
  {
    menuConfig();
    SensorsRead();
    movementLogic();
  }
  else
  {
    if (millis() > currentTimeMenu + TICK_MENU)
    {
      currentTimeMenu = millis();
      SerialBT.println("A. Cambiar Kp ");
      SerialBT.println("B. Cambiar Ki ");
      SerialBT.println("C. Cambiar Kd ");
      SerialBT.println("D. Cambiar Vel Derecha ");
      SerialBT.println("E. Cambiar Vel Izquierda ");
      SerialBT.println("F. Debugear todo ");
<<<<<<< HEAD
      SerialBT.println("G. SALIR ");
    }
  }
  // printAll();
=======
      SerialBT.println("G. NADA ");

    break;
    case KP: if (OpcionSeleccionada == "A") {
      int dato  = SerialBT.read();
      kp = dato;
      }
    break;
    case KI: if (OpcionSeleccionada == "B") {
      int dato  = SerialBT.read();
      ki  = dato;
      }
    break;
    case KD: if (OpcionSeleccionada == "C") {
      int dato  = SerialBT.read();
      kd = dato;
      }
    break;
    case VEL_DERECHA: if (OpcionSeleccionada == "D") {
      int dato  = SerialBT.read(); 
      speedRightPID = dato;
      }
    break;
    case VEL_IZQUIERDA: if (OpcionSeleccionada == "E") {
      int dato  = SerialBT.read();
      speedLeftPID = dato;
      }
    break;
    case DEBUGEAR_TODO: if (OpcionSeleccionada == "F") {
      //bool dato  = SerialBT.read();
      //DEBUG_BUTTON = dato;
      //DEBUG_PID = dato;
      //DEBUG_SENSORS = dato;
      //DEBUG_STATUS = dato;
      }
    break;
    case NADA : 
      SerialBT.println("nada");
    break;
  }
}
*/
void printAll(){
  if (millis() > currentTimeDebugAll + TICK_DEBUG_ALL)
  {
    currentTimeDebugAll = millis();
    if(DEBUG_BUTTON) printButton();
    if (DEBUG_SENSORS) printSensors();
    SerialBT.println("__");
    if (DEBUG_STATUS) printStatus();
    SerialBT.println("_______");
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
>>>>>>> d0b76f3ee8112b5f2127c17d0ac5584b443cd0dd
}