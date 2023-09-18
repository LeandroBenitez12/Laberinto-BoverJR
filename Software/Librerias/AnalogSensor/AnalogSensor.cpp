#include "AnalogSensor.h"

AnalogSensor::AnalogSensor(int p)
{
    pin = p;
    pinMode(pin, INPUT);
}

float AnalogSensor::SensorRead()
{
    long sumReadings = 0;
    for (int i = 0; i < n; i++)
    {
        sumReadings = sumReadings + analogRead(pin);
    }
    float read = sum / n;
    read = read * 1000 / 1024.0;
    return (read);
}
