#ifndef _DRIVER_DRV8833_H
#define _DRIVER_DRV8833_H
#include "Arduino.h"

class EngineESP32 
{
private:
  int speedDerecho;
  int speedIzquierdo;
  // propiedades PWM
  const int frecuencia = 1000;
  const int canalMA1 = 11;
  const int canalMA2 = 12;
  const int canalMB1 = 13;
  const int canalMB2 = 14;
  const int resolucion = 8;
  int pinMA_in1;
  int pinMA_in2;
  int pinMB_in1;
  int pinMB_in2;
public:
    EngineESP32 (int in1, int in2, int in3, int in4);
    void SetSpeed(int vD, int vI);
    void Forward();
    void Backward();
    void Rigth();
    void Left();
    void Stop();
};

#endif