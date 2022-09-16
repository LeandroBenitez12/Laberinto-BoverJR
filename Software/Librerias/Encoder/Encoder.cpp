#include "Encoder.h"

Encoder::Encoder(int p)
{
    pin = p;
    pinMode(pin, INPUT);
}

void Encoder::SetTurnMax(int g360)
{
    vuelta_completa = g360;
}

bool Encoder::FlankDetection()
{
    bool estado_actual = digitalRead(pin);
    bool estado = estado_actual != estado_anterior && estado_actual == HIGH;
    estado_anterior = estado_actual;
    return estado;
}

void Encoder::SetCont(int cont)
{
    contador = cont;
}


float Encoder::Angle()
{
    if (FlankDetection())
    {
        contador++;
        if (contador >= vuelta_completa) contador = 0;
    }
    
    float giro = contador * 360.00 / vuelta_completa;
    return giro;
}
