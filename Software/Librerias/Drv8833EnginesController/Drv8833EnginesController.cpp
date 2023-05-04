#include "Drv8833EnginesController.h"

Engine::Engine(int in1, int in2)
{
    pin_in1 = in1;
    pin_in2 = in2;
    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
}
void Engine::Forward(int pwm)
{
    speed = pwm;
    analogWrite(pin_in1, speed);
    analogWrite(pin_in2, 0);
}
void Engine::Backward(int pwm)
{
    speed = pwm;
    analogWrite(pin_in1, 0);
    analogWrite(pin_in2, speed);
}
void Engine::Stop()
{
    analogWrite(pin_in1, 0);
    analogWrite(pin_in2, 0);
}

EngineController::EngineController(int in1Right, int in2Right, int in1Left, int in2Left)
{
    pinIn1Right = in1Right;
    pinIn2Right = in2Right;
    pinIn1Left = in1Left;
    pinIn2Left = in2Left;
    engineRight = new Engine(pinIn1Right, pinIn2Right);
    engineLeft = new Engine(pinIn1Left, pinIn2Left);
}
void EngineController::Forward(int pwmRight, int pwmLeft)
{
    rightSpeed = pwmRight;
    leftSpeed = pwmLeft;
    engineRight->Forward(rightSpeed);
    engineLeft->Forward(leftSpeed);
}
void EngineController::Backward(int pwmRight, int pwmLeft)
{
    rightSpeed = pwmRight;
    leftSpeed = pwmLeft;
    engineRight->Backward(rightSpeed);
    engineLeft->Backward(leftSpeed);
}
void EngineController::Right(int pwmRight, int pwmLeft)
{
    rightSpeed = pwmRight;
    leftSpeed = pwmLeft;
    engineRight->Backward(rightSpeed);
    engineLeft->Forward(leftSpeed);
}
void EngineController::Left(int pwmRight, int pwmLeft)
{
    rightSpeed = pwmRight;
    leftSpeed = pwmLeft;
    engineRight->Forward(rightSpeed);
    engineLeft->Backward(leftSpeed);
}
void EngineController::Stop()
{
    engineRight->Stop();
    engineLeft->Stop();
}