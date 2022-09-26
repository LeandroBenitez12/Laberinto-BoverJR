#include <Encoder.h>
#include <Motor.h>
#include "BluetoothSerial.h"

//debug
#define DEBUG 1
#define TICK_DEBUG 500
unsigned long tiempo_actual = 0;


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

// Button
#define PIN_BUZZER 23
#define PIN_BUTTON_START 13
bool button_start;

//encoders
#define PIN_ENCODER_RIGHT 23
#define PIN_ENCODER_LEFT 13

//veocidades motores pwm
int speedRight = 180;
int speedLeft = 180;
int averageSpeed = 180;
int speedTurn = 170;
const int PWMChannel1 = 0;
const int PWMChannel2 = 1;

// establecemos conexion bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//instancio los objetos
Motor *engineRigh = new  Motor(PIN_ENGINE_MR1, PIN_ENGINE_MR2, PIN_ENGINE_ENA, PWMChannel1);
Motor *engineLeft = new Motor(PIN_ENGINE_ML1, PIN_ENGINE_ML2, PIN_ENGINE_ENB, PWMChannel2);

Button *start = new Button(PIN_BUTTON_START);

Encoder *EncoderDer = new Encoder(PIN_ENCODER_RIGHT);
Encoder *EncoderIzq = new Encoder(PIN_ENCODER_LEFT);


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

//funcion para doblar por tick del encoder

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
    Stop();

    break;
  }

  case CONTINUE:
  { 
    Forward();
    if (distancia_frontal < DISTANCIA_MINIMA) movement = STOP;
    if (distancia_izquierda > DISTANCIA_LADOS) movement = ANT_TURN;

    break;
  }

  case STOP:
  {
    stop();
    delay(TICK_STOP);

    if (distancia_derecha < DISTANCIA_LADOS && distancia_izquierda > DISTANCIA_LADOS) movement = LEFT_TURN;
    if (distancia_derecha > DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS) movement = RIGHT_TURN;
    if (distancia_derecha > DISTANCIA_LADOS && distancia_izquierda > DISTANCIA_LADOS) movement = LEFT_TURN;
    if (distancia_derecha < DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS) movement = FULL_TURN;

    break;
  }

  case RIGHT_TURN:
  {
    DoblarDer(90);
    movement = POST_TURN;
    
    break;
  }

  case LEFT_TURN:
  {
    DoblarIzq(90);
    movement = POST_TURN;

    break;
  }

  case FULL_TURN:
  {
    DoblarIzq(180);
    movement = POST_TURN;

    break;
  }

  case POST_TURN:
  {
    stop();
    delay(TICK_STOP);
    if(distancia_derecha < DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS) movement = HALLWAY;
    else forward();
    
    break;
  }

  case ANT_TURN:
  {
    stop();
    delay(TICK_STOP);
    Forward();
    delay(TICK_FORWARD);
    movement = LEFT_TURN;
   
    break;
  }
  }
}

void printStatus(movement)
{
  if (millis() > tiempo_actual + TICK_DEBUG)
  {
    String estado_robot = "";
    if (movement == STANDBY) state = "STANDBY";
    else if (movement == CONTINUE) state = "CONTINUE";
    else if (movement == STOP) state = "STOP";
    else if (movement == RIGHT_TURN) state = "RIGHT TURN";
    else if (movement == LEFT_TURN) state = "LEFT TURN";
    else if (movement == FULL_TURN) state = "FULL TURN";
    else if (movement == POST_TURN) state = "POST TURN";
    else if (movement == ANT_TURN) state = "ANT TURN";

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

  if(DEBUG)
  {
    printStatus(movement);
  }

} 
