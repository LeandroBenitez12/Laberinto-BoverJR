#ifndef _QRE1113_H
#define _QRE1113_H
#include "Arduino.h"

class QRE1113
{
  // atributos
private:
  int pin_QRE1113;
public:
  QRE1113(int pin);
  int SensorRead();
};

#endif