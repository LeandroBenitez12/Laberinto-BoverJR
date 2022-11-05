#include "Engine.h"

EngineESP32::EngineESP32(int in1, int in2, int in3, int in4)
{
    pinMA_in1 = in1; 
    pinMA_in2 = in2;    
    pinMB_in1 = in3;    
    pinMB_in2 = in4;
    pinMode(pinMA_in1, OUTPUT);
    pinMode(pinMA_in2, OUTPUT);
    pinMode(pinMB_in1, OUTPUT);
    pinMode(pinMB_in2, OUTPUT);

    ledcSetup(canalMA1, frecuencia, resolucion);
    ledcSetup(canalMA2, frecuencia, resolucion);
    ledcSetup(canalMB1, frecuencia, resolucion);
    ledcSetup(canalMB2, frecuencia, resolucion);

    ledcAttachPin(in1, canalMA1);
    ledcAttachPin(in2, canalMA2);
    ledcAttachPin(in3, canalMB1);
    ledcAttachPin(in4, canalMB2);
}
void EngineESP32::SetSpeed(int vD, int vI)
{
    speedDerecho = vD;
    speedIzquierdo = vI;
}
void EngineESP32::Forward()
{
    ledcWrite(canalMA1, speedDerecho);
    ledcWrite(canalMA2, 0);
    ledcWrite(canalMB1, speedIzquierdo);
    ledcWrite(canalMB2, 0);
}
void EngineESP32::Backward()
{
    ledcWrite(canalMA1, 0 );
    ledcWrite(canalMA2, speedDerecho);
    ledcWrite(canalMB1, 0 );
    ledcWrite(canalMB2, speedIzquierdo);
}
void EngineESP32::Rigth()
{
    ledcWrite(canalMA1, speedDerecho );
    ledcWrite(canalMA2, 0 );
    ledcWrite(canalMB1, 0 );
    ledcWrite(canalMB2, speedIzquierdo);
}
void EngineESP32::Left()
{
    ledcWrite(canalMA1, 0 );
    ledcWrite(canalMA2, speedDerecho);
    ledcWrite(canalMB1, speedIzquierdo);
    ledcWrite(canalMB2, 0 );
}
void EngineESP32::Stop()
{
    ledcWrite(canalMA1, 0 );
    ledcWrite(canalMA2, 0 );
    ledcWrite(canalMB1, 0 );
    ledcWrite(canalMB2, 0 );
}