#include "Engine.h"

Engine::Engine(int in1, int in2)
{
    pin_in1 = in1;
    pin_in2 = in2;
    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
    ledcSetup(canalA, frecuencia, resolucion);
    ledcSetup(canalB, frecuencia, resolucion);
    ledcAttachPin(pin_in1, canalA);
    ledcAttachPin(pin_in2, canalB);
}
void Engine::SetSpeed(int v)
{
    speed = v;
}
void Engine::Forward()
{
    ledcWrite(canalA, speed);
    ledcWrite(canalB, 0);
}
void Engine::Backward()
{
    ledcWrite(canalA, 0);
    ledcWrite(canalB, speed);
}
void Engine::Stop()
{
    ledcWrite(canalA, 0);
    ledcWrite(canalB, 0);
}