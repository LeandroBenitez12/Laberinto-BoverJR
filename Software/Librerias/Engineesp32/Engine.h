#ifndef _DRIVER_DRV8833_H
#define _DRIVER_DRV8833_H
#include "Arduino.h"

class Engine 
{
private:
  int speed = 200;
  // propiedades PWM
  const int frecuencia = 5000;
  const int canalA = 0;
  const int canalB = 1;
  const int resolucion = 8;
  int pin_in1;
  int pin_in2;

public:
    Engine(int in1, int in2);
    void SetSpeed(int v);
    void Forward();
    void Backward();
    void Stop();
};

#endif