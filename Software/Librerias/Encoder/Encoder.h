#ifndef _ENCODER_H
#define _ENCODER_H
#include "Arduino.h"

class Encoder{

private:
    int vuelta_completa = 100;
    int pin;
    bool flanco = HIGH;
    bool estado_anterior = !flanco;
    int contador = 0;

public:
    Encoder(int p);
    void SetTurnMax(int g360);
    void SetCont(int cont);
    bool FlankDetection();
    float Angle();
};

#endif