#include "Encoder.h"

Encoder::Encoder(int p)
{
    pin = p;
    pinMode(pin, INPUT);
}

bool Encoder::FlankDetection()
{
    bool estado_actual = digitalRead(pin);
    bool estado = estado_actual != estado_anterior && estado_actual == HIGH;
    estado_anterior = estado_actual;
    return estado;
}

int Encoder::Cont()
{
    if(FlankDetection()) contador ++;
    return contador;
}

void Encoder::SetCont(int cont)
{
    contador = cont;
}


