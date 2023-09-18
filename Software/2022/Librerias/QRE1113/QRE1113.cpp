#include "QRE1113.h"



QRE1113::QRE1113(int pin)
{
    pin_QRE1113  = pin;
    pinMode(pin_QRE1113,INPUT);
}
int QRE1113::SensorRead()
{
    return analogRead(pin_QRE1113);
}
