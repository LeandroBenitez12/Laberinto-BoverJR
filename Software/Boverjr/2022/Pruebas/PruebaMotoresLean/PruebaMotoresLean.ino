#include <EngineESP32.h>

#define PIN_ENGINE_MR1 23
#define PIN_ENGINE_MR2 22
#define PIN_ENGINE_ML1 19
#define PIN_ENGINE_ML2 18

int speedRight = 50;
int speedLeft = 50;
int averageSpeed = 50;
int speedTurn = 50;

//instancio los objetos
EngineESP32 *engineduo = new  EngineESP32(PIN_ENGINE_MR1, PIN_ENGINE_MR2, PIN_ENGINE_ML1, PIN_ENGINE_ML2);

// funciones de los motores
void forward()
{
  engineduo->SetSpeed(speedRight, speedLeft);
  engineduo->Forward();
  
}

void backward()
{
  engineduo->SetSpeed(speedRight, speedLeft);
  engineduo->Backward();
}

void left()
{
  engineduo->SetSpeed(speedRight, speedLeft);
  engineduo->Left();
}

void right()
{
  engineduo->SetSpeed(speedRight, speedLeft);
  engineduo->Rigth();
  
}

void stop()
{
  engineduo->SetSpeed(0, 0);
  engineduo->Stop();
  
}

void setup() 
{
  
}

void loop() 
{
    forward();
    delay(3000);
    backward();
    delay(3000);
    right();
    delay(3000);
    left();
    delay(3000);
    stop();
    delay(6000);
}
