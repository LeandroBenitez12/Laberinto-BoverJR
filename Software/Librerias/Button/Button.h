#ifndef _BUTTON_LAB_H
#define _BUTTON_LAB_H
#include "Arduino.h"

class Button {
  private:
    int pin;
    bool flank = LOW;
    bool flankKeepPressing = HIGH;
    bool previousState = !flank;
    int cont = 0;

  public:
    Button(int p);
    void SetFlanco(bool f);
    int GetIsPress();
};

#endif