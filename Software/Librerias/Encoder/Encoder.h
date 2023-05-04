#ifndef _ENCODER_H
#define _ENCODER_H
#include "Arduino.h"

class Encoder{

private:
    int pin;
    bool flanco = HIGH;
    bool estado_anterior = !flanco;
    int contador = 0;

public:
    Encoder(int p);
    void SetCont(int cont);
    bool FlankDetection();
    int Cont();
};

#endif