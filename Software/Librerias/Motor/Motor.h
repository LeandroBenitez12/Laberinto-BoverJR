#ifndef _MOTOR_H
#define _MOTOR_H
#include "Arduino.h"

class Motor
{
  // atributos
private:
  int pin_1;
  int pin_2;
  int pin_pwm;
  const int freq = 5000;
  int channel;
  const int resolucion = 8;
  int velocidad = 200;

public:
  Motor(int pin1, int pin2, int pinpwm, int ch);
  // metodos
  void setVelocidad(int vel);
  void forward();
  void backward();
  void stop();
};

#endif