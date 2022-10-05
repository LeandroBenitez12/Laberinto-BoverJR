#include "SharpLab.h"


Sharp::Sharp(int p)
{
    pin = p;
    pinMode(pin, INPUT);
}
double Sharp::SharpDist()
{
    long suma = 0;
    for (int i = 0; i < n; i++) // Realizo un promedio de "n" valores
    {
        suma = suma + analogRead(pin);
    }
    float adc = suma / n;
    
    float distancia_cm = 14660.0 / (adc - 11.0); // hay que ajustar esta formula para que me devuelva bien la distancia
    return (distancia_cm);
}