#include "Ultrasonido.h"

Ultrasonido::Ultrasonido(int trig, int echo)
{
    pin_echo = echo;
    pin_trig = trig;

    pinMode(pin_echo, INPUT);
    pinMode(pin_trig, OUTPUT);
    digitalWrite(pin_trig, LOW);
}
// metodos
int Ultrasonido::SensorRead()
{
    long distancia;
    long duracion;
    // SENSOR
    digitalWrite(pin_trig, HIGH);
    delayMicroseconds(10); // Enviamos un pulso de 10us
    digitalWrite(pin_trig, LOW);
    duracion = pulseIn(pin_echo, HIGH);
    distancia = duracion / 5.82;
    return distancia;
}