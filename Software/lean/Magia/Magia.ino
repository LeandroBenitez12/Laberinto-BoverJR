#include "DriverDRV8833.h"
#include "Pulsador.h"
#include "SharpLab.h"
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

//Sensores de distancia sharp
#define PIN_SENSOR_FRONTAL 35
#define PIN_SENSOR_DERECHA 27
#define PIN_SENSOR_IZQUIERDA 34
double SharpFrontal;
double SharpIzquierdo;
double SharpDerecho;

// Button
#define PIN_BUTTON_START 39
bool button_start;

//Sensor final
#define PIN_SENSOR_FINAL 32
#define TATAMI 300

//veocidades motores pwm
int speedRight = 255;
int speedLeft = 255;
int averageSpeed = 255;
int speedTurn = 255;
int piso_blanco;

//instancio los objetos
Engine *engineRigh = new  Engine(PIN_ENGINE_MR1, PIN_ENGINE_MR2);
Engine *engineLeft = new Engine(PIN_ENGINE_ML1, PIN_ENGINE_ML2);

//Instancio button 
Pulsador*start = new Pulsador(PIN_BUTTON_START);

//Instancio Sensores Sharp
Sharp*sensor_frontal = new Sharp(PIN_SENSOR_FRONTAL)
// Metodos de los motores
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
int movement = STANDBY;
void switchCase(){
    switch(movement){
        case STANDBY:
            button_start = start->GetIsPress();
            if(button_start) movement = CONTINUE;
            else movement = STANDBY;
            break;
        case CONTINUE:
            forward();
            if( piso_blanco < TATAMI) movement = STOP;
            break;
        case STOP:
            stop();
            if( piso_blanco > TATAMI) movement = CONTINUE;
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


void setup() 
{
  Serial.begin(9600);
  pinMode(32,INPUT);
}

void loop() 
{
    SharpFrontal =  sensor_frontal->SharpDist();
    piso_blanco = analogRead(PIN_SENSOR_FINAL);
    switchCase();
    Serial.print(button_start);
    Serial.print("||");
    Serial.println(piso_blanco);
    Serial.print("||");
    Serial.println(SharpFrontal);
}
