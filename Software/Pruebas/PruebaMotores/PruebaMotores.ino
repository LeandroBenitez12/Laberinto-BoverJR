#include <EngineController.h>

#define PIN_RIGHT_ENGINE_IN1 26
#define PIN_RIGHT_ENGINE_IN2 27
#define PIN_LEFT_ENGINE_IN1 18
#define PIN_LEFT_ENGINE_IN2 19
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4
IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *robot = new EngineController(rightEngine, leftEngine);
void setup()
{
}

void loop()
{
    robot->Forward(255);
    delay(500);
    robot->Backward(255);
    delay(500);
    robot->Left(150);
    delay(4000);
    robot->Right(150);
    delay(4000);
    robot->Stop();
    delay(5000);
}
