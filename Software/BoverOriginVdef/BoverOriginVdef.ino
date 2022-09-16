#include <Encoder.h>



#include <Motor.h>



// motores
#define PIN_PWM_ENA 15
#define PIN_MOTOR_MR1 2
#define PIN_MOTOR_MR2 4
#define PIN_PWM_ENB 19
#define PIN_MOTOR_ML1 18
#define PIN_MOTOR_ML2 5

//encoders
#define PIN_ENCODER_DER 23
#define PIN_ENCODER_IZQ 13

// veocidades motores pwm
int velocidad_derecha = 180;
int velocidad_izquierda = 180;
int velocidad_media = 180;
int velocidad_giro = 175;
const int PWMChannel1 = 0;
const int PWMChannel2 = 1;

unsigned long tiempo_actual = 0;
#define TICK 700;

Motor *MDer;
Motor *MIzq;

Encoder *EncoderDer = new Encoder(PIN_ENCODER_DER);
Encoder *EncoderIzq = new Encoder(PIN_ENCODER_IZQ);


// funciones de los motores
void Forward()
{
  MDer->SetVelocidad(velocidad_derecha);
  MIzq->SetVelocidad(velocidad_izquierda);
  MDer->Forward();
  MIzq->Forward();
}
// HACIA ATRAS
void Backward()
{
  MDer->SetVelocidad(velocidad_derecha);
  MIzq->SetVelocidad(velocidad_izquierda);
  MDer->Backward();
  MIzq->Backward();
}
// GIRO SOBRE SU PROPIO EJE A LA DERECHA
void Left()
{
  MDer->SetVelocidad(velocidad_derecha);
  MIzq->SetVelocidad(velocidad_izquierda);
  MDer->Forward();
  MIzq->Backward();
}
// GIRO SOBRE SU PROPIO EJE A LA IZQUIERDA
void Right()
{
  MDer->SetVelocidad(velocidad_derecha);
  MIzq->SetVelocidad(velocidad_izquierda);
  MDer->Backward();
  MIzq->Forward();
}
// PARAR
void Stop()
{
  MDer->SetVelocidad(0);
  MIzq->SetVelocidad(0);
  MDer->Stop();
  MIzq->Stop();
}

void Doblar(int angulo){
    int giro = EncoderDer->Angle();
    while(giro <= angulo) {
      Serial.println("while");
      giro = EncoderDer->Angle();
        }
    EncoderDer->SetCont(0);
    }

void setup() {
  MDer = new  Motor(PIN_MOTOR_MR1, PIN_MOTOR_MR2, PIN_PWM_ENB, PWMChannel1);
  MIzq = new Motor(PIN_MOTOR_ML1, PIN_MOTOR_ML2, PIN_PWM_ENA, PWMChannel2);
  Serial.begin(9600);
  Serial.println("setup");
  
}

void loop() {
  float encDer = EncoderDer->Angle();
  float encIzq = EncoderIzq->Angle();
} 
