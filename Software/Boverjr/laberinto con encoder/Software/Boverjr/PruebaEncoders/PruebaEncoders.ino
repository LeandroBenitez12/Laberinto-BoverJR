
#include <Engine.h>

int interruptPinRight = 26;
int interruptPinLeft = 21;
volatile long rightCont = 0;
volatile long leftCont = 0;
#define TURN_RIGHT_90 17
#define TURN_LEFT_90 20
#define TURN_180 34
#define FORWARD 30 y 35

#define PIN_ENGINE_MR1 23
#define PIN_ENGINE_MR2 22
#define PIN_ENGINE_ML1 19
#define PIN_ENGINE_ML2 18

//veocidades motores pwm
int speedRight = 100;
int speedLeft = 100;
int averageSpeed = 80;
int speedTurn = 80;


Engine *engineRigh = new  Engine(PIN_ENGINE_MR1, PIN_ENGINE_MR2);
Engine *engineLeft = new Engine(PIN_ENGINE_ML1, PIN_ENGINE_ML2);


void rightEncRead()
{
    rightCont++;
}

void leftEncRead()
{
    leftCont++;
}

void turnRight()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < TURN_RIGHT_90 || leftCont < TURN_RIGHT_90)
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
  while(rightCont < TURN_LEFT_90 || leftCont < TURN_LEFT_90)
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
  while(rightCont < TURN_180 && leftCont < TURN_180)
  {
    engineRigh->SetSpeed(speedTurn);
    engineLeft->SetSpeed(speedTurn);
    engineRigh->Forward();
    engineLeft->Backward();
  }
}

void avance()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < FORWARD || leftCont < FORWARD)
  {
    engineRigh->SetSpeed(averageSpeed);
    engineLeft->SetSpeed(averageSpeed);
    engineRigh->Forward();
    engineLeft->Forward();
  }
}

void setup(){
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(interruptPinRight),rightEncRead, RISING);
    attachInterrupt(digitalPinToInterrupt(interruptPinLeft),leftEncRead, RISING);
}

void loop(){
    delay(3000);
    turnLeft();
    engineRigh->Stop();
    engineLeft->Stop();
    delay(2000);
    avance();
    engineRigh->Stop();
    engineLeft->Stop();
    delay(5000);
}
