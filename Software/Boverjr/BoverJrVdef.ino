#include "DriverDRV8833.h"
#include "Button.h"
#include "QRE1113.h"

//debug
#define DEBUG 1
#define TICK_DEBUG 500
unsigned long tiempo_actual = 0;

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
#define PIN_SENSOR_FINAL 32

//veocidades motores pwm
int speedRight = 150;
int speedLeft = 150;
int averageSpeed = 150;
int speedTurn = 130;

// establecemos conexion bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//instancio los objetos
Engine *engineRigh = new  Engine(PIN_ENGINE_MR1, PIN_ENGINE_MR2);
Engine *engineLeft = new Engine(PIN_ENGINE_ML1, PIN_ENGINE_ML2);

Button *start = new Button(PIN_BUTTON_START);

QRE1113 *sensorFinal = new QRE1113(PIN_SENSOR_FINAL);

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
void switchCase()
{
    switch(movement)
    {
        case STANDBY:
            movement = CONTINUE;
        case CONTINUE:
            forward();
            break;
        case STOP:
            stop();
            break;
        case RIGHT_TURN :
            right();
            break;
        case LEFT_TURN :
            left();
            break;
        case FULL_TURN :
            right();
            break;
    }
}
int movement = STANDBY;

void setup() 
{
  SerialBT.begin("Bover");
  Serial.begin(9600);
}

void loop() 
{
    
}