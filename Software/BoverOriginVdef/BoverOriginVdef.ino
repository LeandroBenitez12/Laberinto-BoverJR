#include <Encoder.h>
#include <Motor.h>

// declaramos pines
// motores
#define PIN_PWM_ENA 15
#define PIN_MOTOR_MR1 2
#define PIN_MOTOR_MR2 4
#define PIN_PWM_ENB 19
#define PIN_MOTOR_ML1 18
#define PIN_MOTOR_ML2 5

// ultrasonidos HR04
#define PIN_SENSOR_DERECHO_ECHO1 35
#define PIN_SENSOR_ADELANTE_ECHO2 33
#define PIN_SENSOR_IZQUIERDO_ECHO3 14
#define PIN_SENSOR_DERECHO_TRIG1 32
#define PIN_SENSOR_ADELANTE_TRIG2 25
#define PIN_SENSOR_IZQUIERDO_TRIG3 12
#define TICK_ULTRASONIDO 10
unsigned long tiempo_actual = 0;
unsigned long tiempo_actual_pid = 0;
int distancia_izquierda;
int distancia_frontal;
int distancia_derecha;

// Button
#define PIN_BUZZER 23
#define PIN_BUTTON_START 13
bool boton_start;

//encoders
#define PIN_ENCODER_DER 23
#define PIN_ENCODER_IZQ 13

// const
#define DISTANCIA_MINIMA 200
#define DISTANCIA_LADOS 200
#define DISTANCIA_MAXIMA 250
#define TICK_PID 70
#define TICK_ULTRASONIDO 10
#define TICK_NOTIFICACION 500
#define TICK_STOP 500
#define TICK_FORWARD 500

// veocidades motores pwm
int velocidad_derecha = 180;
int velocidad_izquierda = 180;
int velocidad_media = 180;
int velocidad_giro = 175;
const int PWMChannel1 = 0;
const int PWMChannel2 = 1;

//instancio los objetos

Motor *MDer = new  Motor(PIN_MOTOR_MR1, PIN_MOTOR_MR2, PIN_PWM_ENB, PWMChannel1);
Motor *MIzq = new Motor(PIN_MOTOR_ML1, PIN_MOTOR_ML2, PIN_PWM_ENA, PWMChannel2);

/*Ultrasonido *sensor_frontal = new Ultrasonido(PIN_SENSOR_ADELANTE_TRIG2, PIN_SENSOR_ADELANTE_ECHO2);
Ultrasonido *sensor_derecho = new Ultrasonido(PIN_SENSOR_DERECHO_TRIG1, PIN_SENSOR_DERECHO_ECHO1);
Ultrasonido *sensor_izquierdo = new Ultrasonido(PIN_SENSOR_IZQUIERDO_TRIG3, PIN_SENSOR_IZQUIERDO_ECHO3);



Button *start = new Button(PIN_BUTTON_START);
*/
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

//funcion para doblar por grados
void DoblarDer(int angulo){
    int giro = EncoderDer->Angle();
    while(giro <= angulo) {
      Right();
      giro = EncoderDer->Angle();
        }
    EncoderDer->SetCont(0);
    }

void DoblarIzq(int angulo){
    EncoderDer->SetCont(0);
    int giro = EncoderDer->Angle();
    while(giro <= angulo) {
      Left();
      Serial.println(EncoderDer->Angle());
      giro = EncoderDer->Angle();
        }
    }

// casos de la maquina de estado
enum movimiento
{
  INICIAL,
  PASILLO,
  PARED,
  DESVIO_DERECHA,
  DESVIO_IZQUIERDA,
  CALLEJON,
  POST_DOBLAR,
  ANT_DOBLAR
};

int movimiento = INICIAL;

//logica de movimiento
/*void Movimientos_robot()
{
  switch (movimiento)
  {
  case INICIAL:
  {
    boton_start = start->GetIsPress();
    if (boton_start)
    {
      delay(3000);
      movimiento = PASILLO;
    }
    else
    {
      Stop();
    }

    break;
  }

  case PASILLO:
  { 
    Forward();
    // cambio de caso a pared
    if (distancia_frontal < DISTANCIA_MINIMA) movimiento = PARED;
    if (distancia_izquierda > DISTANCIA_LADOS) movimiento = ANT_DOBLAR;

    break;
  }

  case PARED:
  {
    Stop();
    delay(TICK_STOP);

    if (distancia_derecha < DISTANCIA_LADOS && distancia_izquierda > DISTANCIA_LADOS) movimiento = DESVIO_IZQUIERDA;
    if (distancia_derecha > DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS) movimiento = DESVIO_DERECHA;
    if (distancia_derecha > DISTANCIA_LADOS && distancia_izquierda > DISTANCIA_LADOS) movimiento = DESVIO_IZQUIERDA;
    if (distancia_derecha < DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS) movimiento = CALLEJON;

    break;
  }

  case DESVIO_DERECHA:
  {
    DoblarDer(90);
    movimiento = POST_DOBLAR;
    
    break;
  }

  case DESVIO_IZQUIERDA:
  {
    DoblarIzq(90);
    movimiento = POST_DOBLAR;

    break;
  }

  case CALLEJON:
  {
    DoblarIzq(180);
    movimiento = POST_DOBLAR;

    break;
  }

  case POST_DOBLAR:
  {
    Stop();
    delay(TICK_STOP);
    if(distancia_derecha < DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS) movimiento = PASILLO;
    else Forward();
    
    break;
  }

  case ANT_DOBLAR:
  {
    Stop();
    delay(TICK_STOP);
    Forward();
    delay(TICK_FORWARD);
    movimiento = DESVIO_IZQUIERDA;
   
    break;
  }
  }
}*/
bool a = 1;
void setup() {
  Serial.begin(9600);
  Serial.println("setup");
  
}

void loop() {
  if(a){
    DoblarIzq(3);
    a = 0;
    }
  else {
    Stop();
    Serial.println("Stop");
    }
  
  

} 
