#ifndef _ULTRASONIDO_H
#define _ULTRASONIDO_H
#include "Arduino.h"

class Ultrasonido
{
  // atributos
private:
  int pin_trig;
  int pin_echo;

public:
  Ultrasonido(int trig, int echo);
  // metodos
  int SensorRead();
};

#endif