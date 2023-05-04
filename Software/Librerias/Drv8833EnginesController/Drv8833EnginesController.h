#ifndef _DERV8833_ENGINES_CONTROLLER_H
#define _DERV8833_ENGINES_CONTROLLER_H
#include "Arduino.h"

class Engine 
{
private:
  int pin_in1;
  int pin_in2;
  int speed;

public:
    Engine(int in1, int in2);
    void Forward(int pwm);
    void Backward(int pwm);
    void Stop();
};

class EngineController
{
private:
  int pinIn1Right;
  int pinIn2Right;
  int pinIn1Left;
  int pinIn2Left;
  int rightSpeed;
  int leftSpeed;
  Engine *engineRight;
  Engine *engineLeft;
  

public:
  EngineController(int in1Right, int in2Right, int in1Left, int in2Left);
  void Forward(int pwmRight, int pwmLeft);
  void Backward(int pwmRight, int pwmLeft);
  void Right(int pwmRight, int pwmLeft);
  void Left(int pwmRight, int pwmLeft);
  void Stop();
};
#endif