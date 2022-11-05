#include <Drv8833EnginesController.h>
// motores
#define PIN_ENGINE_MR1 23
#define PIN_ENGINE_MR2 22
#define PIN_ENGINE_ML1 18
#define PIN_ENGINE_ML2 19
#define RIGHT_SPEED 100
#define LEFT_SPEED 100

EngineController *Bover = new EngineController(PIN_ENGINE_MR1, PIN_ENGINE_MR2, PIN_ENGINE_ML1, PIN_ENGINE_ML2);

void setup() {

}

void loop() {
  Bover->Forward(RIGHT_SPEED, LEFT_SPEED);
  delay(3000);
  Bover->Backward(RIGHT_SPEED, LEFT_SPEED);
  delay(3000);
  Bover->Right(RIGHT_SPEED, LEFT_SPEED);
  delay(3000);
  Bover->Left(RIGHT_SPEED, LEFT_SPEED);
  delay(3000);
  Bover->Stop();
  delay(3000);
}