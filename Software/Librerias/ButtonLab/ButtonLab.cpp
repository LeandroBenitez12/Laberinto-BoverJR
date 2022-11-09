#include "ButtonLab.h""

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
            bool actualState = digitalRead(pin);
            cont++;
        }
    if(cont > 800) return 1;
    if(cont < 800) return 2;
    }
}