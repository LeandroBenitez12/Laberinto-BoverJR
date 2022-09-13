#ifndef _ENCODER_H
#define _ENCODER_H
#include "Arduino.h"

class Encoder{

private:
    int vuelta_completa = 100;
    int pin;
    bool flanco = HIGH;
    bool estado_anterior = !flanco;

public:
    Encoder(int p);
    void setTurnMax(int g360);
    bool flankDetection();
    double angle();
};

#endif