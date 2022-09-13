#ifndef _BUTTON_H
#define _BUTTON_H
#include "Arduino.h"

class Pulsador {
  private:
    int pin;
    bool flanco = HIGH;
    bool estado_anterior = !flanco;

  public:
    Pulsador(int p);
    void setFlanco(bool f);
    bool getIsPress();
};

#endif