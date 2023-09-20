#include <Engine.h>
#include "BluetoothSerial.h"

int interruptPinRight = 26;
int interruptPinLeft = 21;
volatile long rightCont = 0;
volatile long leftCont = 0;
#define TURN_90 30
#define TURN_180 60
#define FORWARD 20

#define PIN_ENGINE_MR1 23
#define PIN_ENGINE_MR2 22
#define PIN_ENGINE_ML1 18
#define PIN_ENGINE_ML2 19

//veocidades motores pwm
int speedRight = 100;
int speedLeft = 100;
int averageSpeed = 100;
int speedTurn = 115;

Engine *engineRigh = new  Engine(PIN_ENGINE_MR1, PIN_ENGINE_MR2);
Engine *engineLeft = new Engine(PIN_ENGINE_ML1, PIN_ENGINE_ML2);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

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
  while(rightCont < TURN_90 && leftCont < TURN_90)
  {
    engineRigh->SetSpeed(speedTurn);
    engineLeft->SetSpeed(speedTurn);
    engineRigh->Backward();
    engineLeft->Forward();
    SerialBT.println("turnRight");
    SerialBT.print("rightCont: ");
    SerialBT.print(rightCont);
    SerialBT.print("  //  leftCont: ");   
    SerialBT.println(leftCont);
  }
}

void turnLeft()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < TURN_90 && leftCont < TURN_90)
  {
    engineRigh->SetSpeed(speedTurn);
    engineLeft->SetSpeed(speedTurn);
    engineRigh->Forward();
    engineLeft->Backward();
    SerialBT.println("turnLeft");
    SerialBT.print("rightCont: ");
    SerialBT.print(rightCont);
    SerialBT.print("  //  leftCont: ");   
    SerialBT.println(leftCont);
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
    SerialBT.println("fulTurn");
    SerialBT.print("rightCont: ");
    SerialBT.print(rightCont);
    SerialBT.print("  //  leftCont: ");   
    SerialBT.println(leftCont);
  }
}

void avance()
{
 rightCont = 0;
  leftCont = 0;
  while(rightCont < FORWARD && leftCont < FORWARD)
  {
    engineRigh->SetSpeed(averageSpeed);
    engineLeft->SetSpeed(averageSpeed);
    engineRigh->Forward();
    engineLeft->Forward();
    SerialBT.println("avance");
    SerialBT.print("rightCont: ");
    SerialBT.print(rightCont);
    SerialBT.print("  //  leftCont: ");   
    SerialBT.println(leftCont);
  }
}

void turnRight20()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < 20 && leftCont < 20)
  {
    engineRigh->SetSpeed(speedTurn);
    engineLeft->SetSpeed(speedTurn);
    engineRigh->Backward();
    engineLeft->Forward();
    SerialBT.println("turnRight");
    SerialBT.print("rightCont: ");
    SerialBT.print(rightCont);
    SerialBT.print("  //  leftCont: ");   
    SerialBT.println(leftCont);
  }
}

void turnRight15()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < 15 && leftCont < 15)
  {
    engineRigh->SetSpeed(speedTurn);
    engineLeft->SetSpeed(speedTurn);
    engineRigh->Backward();
    engineLeft->Forward();
    SerialBT.println("turnRight");
    SerialBT.print("rightCont: ");
    SerialBT.print(rightCont);
    SerialBT.print("  //  leftCont: ");   
    SerialBT.println(leftCont);
  }
}

void turnRight10()
{
  rightCont = 0;
  leftCont = 0;
  while(rightCont < 10 && leftCont < 10)
  {
    engineRigh->SetSpeed(speedTurn);
    engineLeft->SetSpeed(speedTurn);
    engineRigh->Backward();
    engineLeft->Forward();
    SerialBT.println("turnRight");
    SerialBT.print("rightCont: ");
    SerialBT.print(rightCont);
    SerialBT.print("  //  leftCont: ");   
    SerialBT.println(leftCont);
  }
}

void setup(){
    Serial.begin(9600);
    SerialBT.begin("Bover");
    attachInterrupt(digitalPinToInterrupt(interruptPinRight),rightEncRead, RISING);
    attachInterrupt(digitalPinToInterrupt(interruptPinLeft),leftEncRead, RISING);
}

void loop(){
    turnRight();
    engineRigh->Stop();
    engineLeft->Stop();
    dalay(5000);
    turnRight20();
    engineRigh->Stop();
    engineLeft->Stop();
    dalay(5000);
    turnRight15();
    engineRigh->Stop();
    engineLeft->Stop();
    dalay(5000);
    turnRight10();
    engineRigh->Stop();
    engineLeft->Stop();
    dalay(5000);
}