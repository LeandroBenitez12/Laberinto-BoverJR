#include "BluetoothSerial.h"
// DEBUG
#define DEBUG_DE_CASOS 1
#define DEBUG_DE_SENSORES 1
#define DEBUG_DE_BUZZER 1
// declaramos pines
// motores
#define ENA 15
#define MR1 5
#define MR2 18
#define ENB 19
#define ML1 2
#define ML2 4
// ultrasonidos
#define ECHO1 35
#define ECHO2 33
#define ECHO3 26
#define ECHO4 14
#define TRIG1 32
#define TRIG2 25
#define TRIG3 27
#define TRIG4 12
#define TICK_STOP 1000
#define BUZZER 23
#define BUTTON 13
// pwm DE UN MOTOR
const int freq = 5000;
const int PWMChannel = 0;
const int resolution = 8;
// pwm DE UN 2DO MOTOR
const int freq2 = 5000;
const int PWMChannel2 = 1;
const int resolution2 = 8;
// Sensores
int sensor_derecho;
int sensor_frontal;
int sensor_izquierdo;
int distancia_izquierda;
int distancia_frontal;
int distancia_derecha;
// button
bool button_start;

// const
#define DISTANCIA_MINIMA 200
#define DISTANCIA_LADOS 200
#define DISTANCIA_MAXIMA 250
#define TICK_PID 70
#define TICK_ULTRASONIDO 10
#define TICK_NOTIFICACION 500
#define TICK_DELAY 500
#define TICK_DOBLAR 399
#define TICK_GIRAR 798

int periodo = 1000;
#define PERIODO 100
unsigned long tiempo_actual = 0;
unsigned long tiempo_actual_stop = 0;
unsigned long tiempo_actual_button = 0;

// veocidades motores pwm
int velocidad_derecha = 180;
int velocidad_izquierda = 180;
int velocidad_media = 180;
int velocidad_giro = 175;



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

// PIN_SENSORES en una matriz
int pin_sensores_ultrasonido[4][2] = {
    {ECHO1, TRIG1},
    {ECHO2, TRIG2},
    {ECHO3, TRIG3},
    {ECHO4, TRIG4},
};
// establecemos conexion bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
//------------------------------------------------------------------------------------------

// ESTADOS DEL SENSOR

enum movimiento
{
  INICIAL,
  PASILLO,
  PARED,
  DESVIO_DERECHA,
  DESVIO_IZQUIERDA,
  CALLEJON,
  POST_DOBLAR,
  ANT_DOBLAR,
};

//------------------------------------------------------------------------------------------

void AsignacionpinesMOTORES()
{
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT_PULLDOWN);
}
// Asignacion de pines utilizando la matriz para no declarar cada sensor y hacer 100 lines de cod
void asignacionPinesSensores()
{
  for (int fila = 0; fila < 4; fila++)
  {
    int pin_echo = pin_sensores_ultrasonido[fila][0];
    int pin_trig = pin_sensores_ultrasonido[fila][1];
    pinMode(pin_echo, INPUT);
    pinMode(pin_trig, OUTPUT);
    digitalWrite(pin_trig, LOW);
  }
}
void AsignacionPinesPWM()
{
  ledcSetup(PWMChannel, freq, resolution);
  ledcSetup(PWMChannel2, freq2, resolution2);
  ledcAttachPin(ENA, PWMChannel);
  ledcAttachPin(ENB, PWMChannel2);
}
//---------------------------------------------------------------------------------------------------
enum MOTOR_ESTADO
{
  AVANZAR,
  RETROCEDER,
  STOP
};
void Motor(int estado, int pin1, int pin2, int pwm, int velocidad)
{

  ledcWrite(pwm, velocidad);

  switch (estado)
  {
  case AVANZAR:
  {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    break;
  }
  case RETROCEDER:
  {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    break;
  }
  case STOP:
  {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    break;
  }
  }
}

void MotorDer(int estado)
{
  Motor(estado, MR1, MR2, PWMChannel,velocidad_derecha);
}
void MotorIzq(int estado)
{
  Motor(estado, ML1, ML2, PWMChannel2,velocidad_izquierda);
}
// HACIA ADELANTE
void Forward()
{
  MotorDer(AVANZAR);
  MotorIzq(AVANZAR);
}
// HACIA ATRAS
void Backward()
{
  MotorDer(RETROCEDER);
  MotorIzq(RETROCEDER);
}
// GIRO SOBRE SU PROPIO EJE A LA DERECHA
void Right()
{
  MotorDer(RETROCEDER);
  MotorIzq(AVANZAR);
}
// GIRO SOBRE SU PROPIO EJE A LA IZQUIERDA
void Left()
{
  MotorDer(AVANZAR);
  MotorIzq(RETROCEDER);
}
// PARAR
void Stop()
{
  MotorDer(STOP);
  MotorIzq(STOP);
}
void Buzzer()
{
  digitalWrite(BUZZER, HIGH);
}
//-------------------------------------------------------------
int LeerUltrasonidos(int pin_trig, int pin_echo)
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
int LeerButton(int button)
{
  bool estado_button;
  estado_button = digitalRead(button);
  return estado_button;
}

//-------------------------------------------------------------
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
  SerialBT.print(button_start);
  SerialBT.println(" | ");
}
void ImprimirDatos(int sd, int sai, int si)
{
  SerialBT.print("   distancia_derecha: ");
  SerialBT.print(sd);
  SerialBT.print("   - distancia_frotal: ");
  SerialBT.print(sai);
  SerialBT.print("   - distancias_izquierda: ");
  SerialBT.println(si);
}

//----------------------------------------------------------------
// Maquina de estados
int movimiento = INICIAL;
// maquina de estado
void Movimientos_robot()
{
  switch (movimiento)
  {
  case INICIAL:
    {
    if (millis() > tiempo_actual_button + PERIODO)
    {
      tiempo_actual_button = millis();
      button_start = LeerButton(BUTTON);
    }
    if (button_start)
    {
      {movimiento = PASILLO;}
    }
    else
      {
        Stop();
      }
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
    else if(distancia_izquierda > DISTANCIA_LADOS){
      movimiento = ANT_DOBLAR;
    }

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
    velocidad_derecha = velocidad_giro;
    velocidad_izquierda = velocidad_giro;
    delay(TICK_DELAY);
    Right();
    delay(TICK_DOBLAR);
    movimiento = POST_DOBLAR;

    break;
  }

  case DESVIO_IZQUIERDA:
  {
    velocidad_derecha = velocidad_giro;
    velocidad_izquierda = velocidad_giro;
    Left();
    delay(TICK_DOBLAR);
    movimiento = POST_DOBLAR;

    break;
  }

  case CALLEJON:
  {
    velocidad_derecha = velocidad_giro;
    velocidad_izquierda = velocidad_giro;
    Left();
    delay(TICK_GIRAR);
    movimiento = POST_DOBLAR;

    break;
  }

  case POST_DOBLAR:
  {
    Forward();
    delay(TICK_DELAY);
    Stop();
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

//-------------------------------------------------------------
void setup()
{
  SerialBT.begin("Bover"); // Nombre del Bluetooth
  AsignacionpinesMOTORES();
  asignacionPinesSensores();
  AsignacionPinesPWM();
}
void loop()
{
  if (Serial.available())
  {
    SerialBT.write(Serial.read());
  }

  if (millis() > tiempo_actual + TICK_ULTRASONIDO)
  {
    tiempo_actual = millis();
    distancia_izquierda = LeerUltrasonidos(TRIG1, ECHO1);
    distancia_frontal = LeerUltrasonidos(TRIG2, ECHO2);
    distancia_derecha = LeerUltrasonidos(TRIG4, ECHO4);
  }
  Movimientos_robot();
  if (SerialBT.available())
  {

    if (DEBUG_DE_CASOS)
    {
      imprimir_casos(movimiento);
    }
    if (DEBUG_DE_SENSORES)
    {
      ImprimirDatos(distancia_derecha, distancia_frontal, distancia_izquierda);
    }
  }
}