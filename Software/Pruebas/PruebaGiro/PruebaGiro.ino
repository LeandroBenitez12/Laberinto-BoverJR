#include <EngineController.h>

#define PIN_RIGHT_ENGINE_IN1 26
#define PIN_RIGHT_ENGINE_IN2 27
#define PIN_LEFT_ENGINE_IN1 18
#define PIN_LEFT_ENGINE_IN2 19
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4

//Constantes TICKS , VARIABLES VELOCIDADES
#define TICK_GIRO_90
#define TICK_GIRO_180
#define TICK
#define TICK


//Boton
#define PIN_BUTTON_START 32
bool stateButton = 0;
bool wall;


IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *robot = new EngineController(rightEngine, leftEngine);

Button *buttonStart1 = new Button(PIN_BUTTON_START);

void turnRight(){
  Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_90_DELAY);
}

void turnLeft(){
  Bover->Left(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_90_DELAY);
}

void postTurn(){
  Bover->Forward(210, 190);
  delay(ENTRAR_EN_PASILLO);
}

void fullTurn(){
  Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(GIROS_180_DELAY);
} 

void ignoreTurn(){
  Bover->Forward(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
  delay(ENTRAR_EN_PASILLO);
}

testFull(){
    stateButton = 
}

void setup()
{
}

void loop()
{

}
