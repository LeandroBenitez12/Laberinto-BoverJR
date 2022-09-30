#include "Engine.h"

Engine::Engine(int in1, int in2)
{
    pin_in1 = in1;
    pin_in2 = in2;

    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
}
void Engine::SetSpeed(int v)
{
    speed = v;
}
void Engine::Forward()
{
    analogWrite(pin_in1, speed);
    analogWrite(pin_in2, 0);
}
void Engine::Backward()
{
    analogWrite(pin_in1, 0);
    analogWrite(pin_in2, speed);
}
void Engine::Stop()
{
    analogWrite(pin_in1, 0);
    analogWrite(pin_in2, 0);
}