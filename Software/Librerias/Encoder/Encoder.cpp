#include "Encoder.h"

Encoder::Encoder(int p)
{
    pin = p;
    pinMode(pin, INPUT);
}

void Encoder::setTurnMax(int g360)
{
    vuelta_completa = g360;
}

bool Encoder::flankDetection()
{
    bool estado_actual = digitalRead(pin);
    bool estado = estado_actual != estado_anterior && estado_actual == HIGH;
    estado_anterior = estado_actual;
    return estado;
}

double Encoder::angle()
{
    int contador = 0;
    if (flankDetection())
    {
        contador++;
        if (contador >= vuelta_completa)
            contador = 0;
    }
    float giro = contador * 360 / vuelta_completa;
    return giro;
}