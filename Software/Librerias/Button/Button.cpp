#include "Button.h"

Button::Button(int p)
{
    pin = p;

    pinMode(pin, INPUT);
}

void Button::SetFlanco(bool f)
{
    flanco = f;
    estado_anterior = !flanco;
}

bool Button::GetIsPress()
{
    bool estado_actual = digitalRead(pin);
    bool estado = (estado_anterior != estado_actual) && estado_actual == flanco;
    estado_anterior = estado_actual;
    return estado;
}