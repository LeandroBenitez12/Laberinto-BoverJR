#ifndef _BUTTON_H
#define _BUTTON_H
#include "Arduino.h"

class Button {
  private:
    int pin;
    bool flanco = HIGH;
    bool estado_anterior = !flanco;

  public:
    Button(int p);
    void SetFlanco(bool f);
    bool GetIsPress();
};

#endif