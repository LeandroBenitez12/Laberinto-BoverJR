#include "BluetoothSerial.h"
// DEBUG
#define DEBUG_DE_CASOS 0
#define DEBUG_SENSORES 0
#define DEBUG_VELOCIDAD 0
#define DEBUG_PID 0
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
// encoders
#define PIN_ENCODER_DER
#define PIN_ENCODER_IZQ
float vuelta_completa = 100;
int contador = 0;



// Button
#define PIN_BUZZER 23
#define PIN_BUTTON_START 13
bool boton_start;
// const
#define DISTANCIA_MINIMA 200
#define DISTANCIA_LADOS 200
#define DISTANCIA_MAXIMA 250
#define TICK_PID 70
#define TICK_ULTRASONIDO 10
#define TICK_NOTIFICACION 500
#define TICK_DELAY 500

// veocidades motores pwm
int velocidad_derecha = 180;
int velocidad_izquierda = 180;
int velocidad_media = 180;
int velocidad_giro = 175;
const int PWMChannel1 = 0;
const int PWMChannel2 = 1;

// PID
//------------------------------------------------------------------------------------------
double kp = 0.1721;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double Input, output, setPoint = 0;
double deltaError;
double PID1;

double computePID(double input)
{
  currentTime = millis();                             // get current time
  elapsedTime = (double)(currentTime - previousTime); // compute time elapsed from previous computation
  error = input - setPoint;
  deltaError = error - lastError;
  double out = kp * error + kd * deltaError;
  lastError = error;          // remember current error
  previousTime = currentTime; // remember current time

  return out; // have function return the PID output
}
//------------------------------------------------------------------------------------------
// establecemos conexion bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

class Motor
{
  // atributos
private:
  int pin_1;
  int pin_2;
  int pin_pwm;
  const int freq = 5000;
  int channel;
  const int resolucion = 8;
  int velocidad = 200;

public:
  Motor(int pin1, int pin2, int pinpwm, int ch)
  {
    pin_1 = pin1;
    pin_2 = pin2;
    pin_pwm = pinpwm;
    channel = ch;

    ledcSetup(channel, freq, resolucion);
    ledcAttachPin(pin_pwm, channel);

    pinMode(pin_1, OUTPUT);
    pinMode(pin_2, OUTPUT);
  }
  // metodos
  int setVelocidad(int vel)
  {
    velocidad = vel;
  }
  void ADELANTE()
  {
    ledcWrite(channel, velocidad);
    digitalWrite(pin_1, HIGH);
    digitalWrite(pin_2, LOW);
  }
  void ATRAS()
  {
    ledcWrite(channel, velocidad);
    digitalWrite(pin_1, LOW);
    digitalWrite(pin_2, HIGH);
  }
  void PARAR()
  {
    ledcWrite(channel, velocidad);
    digitalWrite(pin_1, LOW);
    digitalWrite(pin_2, LOW);
  }
};


class Ultrasonido
{
  // atributos
private:
  int pin_trig;
  int pin_echo;

public:
  Ultrasonido(int trig, int echo)
  {
    pin_echo = echo;
    pin_trig = trig;

    pinMode(pin_echo, INPUT);
    pinMode(pin_trig, OUTPUT);
    digitalWrite(pin_trig, LOW);
  }
  // metodos
  int LeerUltrasonidos()
  {
    long distancia;
    long duracion;
    // SENSOR
    digitalWrite(pin_trig, HIGH);
    delayMicroseconds(10); // Enviamos un pulso de 10us
    digitalWrite(pin_trig, LOW);
    duracion = pulseIn(pin_echo, HIGH);
    distancia = duracion / 5.82;
    return distancia;
  }
};

class Buzzer
{
private:
  int pin;

public:
  Buzzer(int p)
  {
    pin = p;
    pinMode(pin, OUTPUT);
  }
  void setPrenderBuzzer()
  {
    digitalWrite(pin, HIGH);
  }
  void setApagarBuzzer()
  {
    digitalWrite(pin, LOW);
  }
};
class Button
{
private:
  int pin = 9;

  // metodo
public:
  Button(int p)
  {
    pin = p;

    pinMode(pin, INPUT_PULLDOWN);
  }

  // metodos o acciones
  bool getIsPress()
  {
    bool estado = digitalRead(pin);
    return estado;
  }
};

class EncoderInflarrojo{

private:
    int pin;
    bool flanco = HIGH;
    bool estado_anterior = !flanco;

public:

     EncoderInflarrojo(int p){
        
        pin = p;
        pinMode(pin,INPUT);
    }

//funcion para la deteccion del cambio de flanco
     bool DeteccionFlanco() {
     bool estado_actual = digitalRead(pin);
     bool estado = estado_actual != estado_anterior && estado_actual == HIGH;
     estado_anterior = estado_actual;
     return estado;
}

//funcion que cuenta los cambios de flancos
int Contador(){
  
  contador++;

  if(contador>=vuelta_completa)contador=0;
  return contador;

  }
  
//cuento los grados del giro
double Giro(){
  if (DeteccionFlanco()) {
      contador = Contador();
    }
  float giro = contador*360/vuelta_completa;
  return giro;
  }

// intancio los ultrasonidos
Ultrasonido sensor_frontal = Ultrasonido(PIN_SENSOR_ADELANTE_TRIG2, PIN_SENSOR_ADELANTE_ECHO2);
Ultrasonido sensor_derecho = Ultrasonido(PIN_SENSOR_DERECHO_TRIG1, PIN_SENSOR_DERECHO_ECHO1);
Ultrasonido sensor_izquierdo = Ultrasonido(PIN_SENSOR_IZQUIERDO_TRIG3, PIN_SENSOR_IZQUIERDO_ECHO3);

// instancio los motores

Motor *MDer;
Motor *MIzq;

// Instancio los buttons *punteros
Button *start = new Button(PIN_BUTTON_START);

// Instancio los buzzers
Buzzer *b1 = new Buzzer(PIN_BUZZER);

// Instancio los encoders
EncoderInflarrojo *encoder_der = new EncoderInflarrojo(PIN_ENCODER_DER);
EncoderInflarrojo *encoder_izq = new EncoderInflarrojo(PIN_ENCODER_IZQ);

// funciones de los motores
void Forward()
{
  MDer->setVelocidad(velocidad_derecha);
  MIzq->setVelocidad(velocidad_izquierda);
  MDer->ADELANTE();
  MIzq->ADELANTE();
}
// HACIA ATRAS
void Backward()
{
  MDer->setVelocidad(velocidad_derecha);
  MIzq->setVelocidad(velocidad_izquierda);
  MDer->ATRAS();
  MIzq->ATRAS();
}
// GIRO SOBRE SU PROPIO EJE A LA DERECHA
void Left()
{
  MDer->setVelocidad(velocidad_derecha);
  MIzq->setVelocidad(velocidad_izquierda);
  MDer->ADELANTE();
  MIzq->ATRAS();
}
// GIRO SOBRE SU PROPIO EJE A LA IZQUIERDA
void Right()
{
  MDer->setVelocidad(velocidad_derecha);
  MIzq->setVelocidad(velocidad_izquierda);
  MDer->ATRAS();
  MIzq->ADELANTE();
}
// PARAR
void Stop()
{
  MDer->setVelocidad(0);
  MIzq->setVelocidad(0);
  MDer->PARAR();
  MIzq->PARAR();
}

//FUNCIONES PARA GIRAR POR ANGULO
void DoblarIzq(int angulo){

    while(encoder_der->Giro()>=angulo) {
        MDer->setVelocidad(velocidad_giro);
        MDer->ADELANTE();
        }
    while(encoder_izq->Giro()>=angulo) {
        Mizq->setVelocidad(velocidad_giro);
        Mizq->ATRAS();
        }
    }

void DoblarDer(int angulo){

    while(encoder_der->Giro()>=angulo) {
        MDer->setVelocidad(velocidad_giro);
        MDer->ATRAS();
        }
    while(encoder_izq->Giro()>=angulo) {
        Mizq->setVelocidad(velocidad_giro);
        Mizq->ADELANTE();
        }
    }

// buzzers
void BuzzerOn()
{
  b1->setPrenderBuzzer();
}
void BuzzerOff()
{
  b1->setApagarBuzzer();
}

// imprimir distancias
void imprimir_distancia()
{
  SerialBT.print("distancia_frontal");
  SerialBT.println(distancia_frontal);
  SerialBT.print("distancia_izquierda");
  SerialBT.println(distancia_izquierda);
  SerialBT.print("distancia_derecha");
  SerialBT.println(distancia_derecha);
}
//-------------------------------------------------------------
// casos de la maquina de estado
enum movimiento
{
  INICIAL,
  PASILLO,
  PARED,
  DESVIO_DERECHA,
  DESVIO_IZQUIERDA,
  CALLEJON,
  POST_DOBLAR
};
void imprimir_casos(int ubicacion)
{
  String estado_robot = "";
  if (ubicacion == INICIAL)
  {
    estado_robot = "sta off";
  }
  else if (ubicacion == PASILLO)
  {
    estado_robot = "segui noma";
  }
  else if (ubicacion == PARED)
  {
    estado_robot = "Hay pared";
  }
  else if (ubicacion == DESVIO_IZQUIERDA)
  {
    estado_robot = "DOBLAR a la IZQUIERDA";
  }
  else if (ubicacion == DESVIO_DERECHA)
  {
    estado_robot = " doblar a la derecha";
  }
  else if (ubicacion == CALLEJON)
  {
    estado_robot = "REGRESAR";
  }
  else if (ubicacion == POST_DOBLAR)
  {
    estado_robot = "POST_DOBLAR";
  }

  SerialBT.print("State: ");
  SerialBT.print(estado_robot);
  SerialBT.print(" | ");
  SerialBT.print(boton_start);
  SerialBT.println(" | ");
}
void imprimir_velocidad()
{
  SerialBT.print(velocidad_derecha);
  SerialBT.print(" | ");
  SerialBT.println(velocidad_izquierda);
}

int movimiento = INICIAL;
// maquina de estado
void Movimientos_robot()
{
  switch (movimiento)
  {
  case INICIAL:
  {
    boton_start = start->getIsPress();
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
    
    // PID
    // asi el pid fira bien en los sentidos tipo si esta cerca de la pared derecha va hacia la izquierda
    if (velocidad_izquierda < 155)
        velocidad_izquierda = 155;
      if (velocidad_derecha < 155)
        velocidad_derecha = 155;

    if (PID1)
    {
        if (PID1 > 30)
        PID1 = 30;
        if (PID1 < -30)
        PID1 = -30;

      velocidad_derecha = (velocidad_media - (PID1));
      velocidad_izquierda = (velocidad_media + (PID1));

      
      if (velocidad_izquierda < 160)
        velocidad_izquierda = 160;
      if (velocidad_derecha < 160)
        velocidad_derecha = 160;
    }
    Forward();
    // cambio de caso a pared
    if (distancia_frontal < DISTANCIA_MINIMA)
      movimiento = PARED;

    break;
  }

  case PARED:
  {
    Stop();
    delay(TICK_DELAY);

    if (distancia_derecha < DISTANCIA_LADOS && distancia_izquierda > DISTANCIA_LADOS)
      movimiento = DESVIO_IZQUIERDA;
    if (distancia_derecha > DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS)
      movimiento = DESVIO_DERECHA;
    if (distancia_derecha > DISTANCIA_LADOS && distancia_izquierda > DISTANCIA_LADOS)
      movimiento = DESVIO_IZQUIERDA;
    if (distancia_derecha < DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS)
      movimiento = CALLEJON;

    break;
  }

  case DESVIO_DERECHA:
  {
    DoblarDer(90);
    delay(TICK_DELAY);
    movimiento = POST_DOBLAR;
    
    break;
  }

  case DESVIO_IZQUIERDA:
  {
    DoblarIzq(90);
    delay(TICK_DELAY);
    movimiento = POST_DOBLAR;

    break;
  }

  case CALLEJON:
  {
    DoblarIzq(180);
    delay(TICK_DELAY);
    movimiento = POST_DOBLAR;

    break;
  }

  case POST_DOBLAR:
  {
    Stop();
    delay(TICK_DELAY);
    Forward();
    delay(TICK_DELAY);
    movimiento = PASILLO;
    
    break;
  }

  case ANT_DOBLAR:
  {
    Stop();
    delay(TICK_DELAY);
    Forward();
    delay(TICK_DELAY);
    movimiento = DESVIO_IZQUIERDA;
   
    break;
  }
  }
}

void setup()
{ 
  MDer = new  Motor(PIN_MOTOR_MR1, PIN_MOTOR_MR2, PIN_PWM_ENB, PWMChannel1);
  MIzq = new Motor(PIN_MOTOR_ML1, PIN_MOTOR_ML2, PIN_PWM_ENA, PWMChannel2);
  Serial.begin(9600);
  SerialBT.begin("Bover"); 
}

void loop()
{
  movimiento = PASILLO;
  Input = distancia_derecha - distancia_izquierda;
  if (millis() > tiempo_actual_pid + TICK_PID)
    {
      tiempo_actual_pid = millis();
      PID1 = computePID(Input);
    }
  if (millis() > tiempo_actual + TICK_ULTRASONIDO)
    {
      tiempo_actual = millis();
      distancia_frontal = sensor_frontal.LeerUltrasonidos();
      distancia_izquierda = sensor_izquierdo.LeerUltrasonidos();
      distancia_derecha = sensor_derecho.LeerUltrasonidos();

    }

  Movimientos_robot();
  if (DEBUG_SENSORES)
  {
    imprimir_distancia();
  }
  if (DEBUG_DE_CASOS)
  {
    imprimir_casos(movimiento);
  }

  if (DEBUG_VELOCIDAD)
  {
    imprimir_velocidad();
  }

}
