#ifndef _BUTTON_H
#define _BUTTON_H
#include "Arduino.h"

class Button {
  private:
    int pin;
    bool flank = HIGH;
    bool previousState = !flank;
    int cont = 0;
  public:
    Button(int p);
    void SetFlanco(bool f);
    int GetIsPress();
};

#endif