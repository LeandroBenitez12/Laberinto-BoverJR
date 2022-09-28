#ifndef _DRIVER_DRV8833_H
#define _DRIVER_DRV8833_H
#include "Arduino.h"

class Engine {
private:
  int speed = 200;
  int pin_in1;
  int pin_in2;

public:
    Engine(int in1, int in2);
    void SetVelocidad(int v);
    void Forward();
    void Backward();
    void Stop();
};

#endif