#include "ButtonLab.h"

Button::Button(int p)
{
    pin = p;
    pinMode(pin, INPUT);
}

void Button::SetFlanco(bool f)
{
    flank = f;
    previousState = !flank;
}

int Button::GetIsPress()
{
    bool actualState = digitalRead(pin);
    bool State = (previousState != actualState) && actualState == flank;
    previousState = actualState;
    if(State)
    {
      while(actualState)
      {
        actualState = digitalRead(pin);
        cont = cont++;
      }
      if(cont > 700)
        return 1;
      else if(cont < 700)
        return 2;
    }   
}