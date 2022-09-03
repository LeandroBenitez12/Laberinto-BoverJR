#include "BluetoothSerial.h"
// DEBUG
#define DEBUG_DE_CASOS 0
#define DEBUG_SENSORES 1
#define DEBUG_PID 0
// declaramos pines
// motores
#define ENA 15
#define MR1 18
#define MR2 5
#define ENB 19
#define ML1 4
#define ML2 2
// ultrasonidos HR04
#define ECHO1 35
#define ECHO2 33
#define ECHO3 26
#define ECHO4 14
#define TRIG1 32
#define TRIG2 25
#define TRIG3 27 
#define TRIG4 12
#define TICK_ULTRASONIDO 10
unsigned long tiempo_actual = 0;
int distancia_izquierda;
int distancia_frontal;
int distancia_derecha;

// Button
#define PIN_BUZZER 23
#define PIN_BUTTON_START 13

// const
#define DISTANCIA_MINIMA 200
#define DISTANCIA_LADOS 100
#define DISTANCIA_MAXIMA 250
#define TICK_PID 70
#define TICK_ULTRASONIDO 10
#define TICK_NOTIFICACION 500
#define TICK_DELAY 500
#define TICK_DOBLAR 399
#define TICK_GIRAR 798

// veocidades motores pwm
int velocidad_derecha = 200;
int velocidad_izquierda = 200;
int velocidad_media = 200;
int velocidad_giro = 188;
// pwm DE UN MOTOR
const int freq1 = 5000;
const int PWMChannel1 = 0; // LOS CANALES PWM TIENEN QUE SER DISTINTOS!!!!!!
const int resolution1 = 8;
// pwm DE UN 2DO MOTOR
const int freq2 = 5000;
const int PWMChannel2 = 1;
const int resolution2 = 8;

class Motor
{
    //atributos
private:   
    int pin_1;
    int pin_2;
    int pin_pwm;
    const int freqpwm = 5000;
    int channel;
    const int resolucion = 8;
    int velocidad = 200; 
public:

    Motor(int pin1, int pin2, int pinpwm, int pwm){
        pin_1 = pin1;
        pin_2 = pin2;
        pin_pwm = pinpwm;
        channel = pwm;

        ledcSetup(channel, freq, resolucion);
	      ledcAttachPin(pin_pwm, channel);

        pinMode(pin_1, OUTPUT);
        pinMode(pin_2, OUTPUT);

    	
    }
//metodos
    void Forward (){
        digitalWrite(pin_1, HIGH);
        digitalWrite(pin_2, LOW);
        ledcWrite(pin_pwm, velocidad);
    }
    void Backward (){
        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, HIGH);
        ledcWrite(pin_pwm, velocidad);
    }
    void Stop (){
        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, LOW);
        ledcWrite(pin_pwm, velocidad);
    }        
};

class Ultrasonido{
    //atributos
    private:
        int pin_trig;
        int pin_echo;

    public:
        Ultrasonido(int trig, int echo){
        pin_echo = echo;
        pin_trig = trig;

        pinMode(pin_echo, INPUT);
        pinMode(pin_trig, OUTPUT);
        digitalWrite(pin_trig, LOW);
    }
    //metodos
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

class Buzzer {
  private:
    int pin;

  public:
    Buzzer(int p) {
      pin = p;
      pinMode(pin, OUTPUT);
    }
    void setPrenderBuzzer() {
      digitalWrite(pin, HIGH);
    }
    void setApagarBuzzer() {
      digitalWrite(pin, LOW);
    }

};
class Button {
  private:
    int pin = 9;
    bool state = HIGH;

    //metodo
  public:
    Button(int p) {
      pin = p;

      pinMode(pin, INPUT_PULLUP);
    }

    //metodos o acciones
    bool getIsPress() {
      bool estado = digitalRead(pin);
      return estado;
    }

};

//intancio los ultrasonidos
Ultrasonido sensor_frontal = Ultrasonido(TRIG2, ECHO2);
Ultrasonido sensor_derecho = Ultrasonido(TRIG1, ECHO1);
Ultrasonido sensor_izquierdo = Ultrasonido(TRIG4, ECHO4);

//instancio los motores

Motor mDer = Motor(MR1, MR2, ENA, PWMChannel1);
Motor mIzq = Motor(ML1, ML2, ENB, PWMChannel2);

//Instancio los buttons *punteros
Button *start = new  Button(PIN_BUTTON_START);

//Instancio los buzzers
Buzzer *b1 = new Buzzer(PIN_BUZZER);

//funciones de los motores
void Forward()
{
  MDer(AVANZAR);
  MIzq(AVANZAR);
}
// HACIA ATRAS
void RightBackward()
{
  MDer(RETROCEDER);
  MIzq(RETROCEDER);
}
// GIRO SOBRE SU PROPIO EJE A LA DERECHA
void Left()
{
  MDer(RETROCEDER);
  MIzq(AVANZAR);
}
// GIRO SOBRE SU PROPIO EJE A LA IZQUIERDA
void Right()
{
  MDer(AVANZAR);
  MIzq(RETROCEDER);
}
// PARAR
void Stop()
{
  MotorDer(STOP);
  MotorIzq(STOP);
}

//buzzers
void BuzzerOn(){
    b1->setPrenderBuzzer(); 
}
void BuzzerOff(){
    b1->setApagarBuzzer();
}

//imprimir distancias
void imprimir_distancia(){
    Serial.print("distancia_frontal"  );
    Serial.println(distancia_frontal);
    Serial.print("distancia_izquierda"  );
    Serial.println(distancia_izquierda);
    Serial.print("distancia_derecha"  );
    Serial.println(distancia_derecha);
}

//casos de la maquina de estado
enum MOVIMIENTOS{
  INICIAL,
  PASILLO,
  PARED,
  DESVIO_DERECHA,
  DESVIO_IZQUIERDA,
  CALLEJON,
  POST_DOBLAR
};

//maquina de estado
void Movimientos_robot() {
switch (movimiento) {
  case INICIAL: {
    bool boton_start = start->getIsPress();
    if (!boton_start) {
    movimiento = PASILLO; 
    }
    else
    {
      Stop();
    }
    
     break;
  }

  case PASILLO: {
    //PID
    //asi el pid fira bien en los sentidos tipo si esta cerca de la pared derecha va hacia la izquierda
    double Input = distancia_derecha - distancia_izquierda;

    if (millis() > tiempo_actual_pid + TICK_PID) {
      tiempo_actual_pid = millis();
      PID1 = computePID(Input);
    }
    
    if(PID) {
    velocidad_derecha = (velocidad_media - (PID1));
    velocidad_izquierda = (velocidad_media + (PID1));
    
    if (PID1 > 30) PID1 =  30;
    if (PID1 < -30) PID1 = -30;

    if (velocidad_izquierda < 155) velocidad_izquierda = 155;
    if (velocidad_derecha < 155) velocidad_derecha = 155;
    }

    //cambio de caso a pared
    if(distancia_frontal < DISTANCIA_MINIMA) movimiento = PARED;

    break;
  }

  case PARED: {
    Stop();
    delay(TICK_DELAY);
    
    if(distancia_derecha < DISTANCIA_LADOS && distancia_izquierda > DISTANCIA_LADOS) movimiento = DESVIO_IZQUIERDA;
    if(distancia_derecha > DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS) movimiento = DESVIO_DERECHA;
    if(distancia_derecha > DISTANCIA_LADOS && distancia_izquierda > DISTANCIA_LADOS) movimiento = DESVIO_IZQUIERDA;
    if(distancia_derecha < DISTANCIA_LADOS && distancia_izquierda < DISTANCIA_LADOS) movimiento = CALLEJON;
    
    break;
  }

  case DESVIO_DERECHA: {
    velocidad_derecha = velocidad_giro;
    velocidad_izquierda = velocidad_giro;
    delay(TICK_DELAY);
    Right();
    delay(TICK_DOBLAR);
    movimiento = POST_DOBLAR;
    
    break;
  }

  case DESVIO_IZQUIERDA: {
    velocidad_derecha = velocidad_giro;
    velocidad_izquierda = velocidad_giro;
    delay(TICK_DELAY);
    Left();
    delay(TICK_DOBLAR);
    movimiento = POST_DOBLAR;
    
    break;
  }

  case CALLEJON: {
    velocidad_derecha = velocidad_giro;
    velocidad_izquierda = velocidad_giro;
    delay(TICK_DELAY);
    Left();
    delay(TICK_GIRAR);
    movimiento = POST_DOBLAR;
    
    break;
  }

  case POST_DOBLAR: {
    Stop();
    delay(TICK_DELAY);
    Forward();
    delay(TICK_DELAY);
    movimiento = PASILLO;
    break;
  }
}
}

// PID
//------------------------------------------------------------------------------------------
double kp = 0.3;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint = 0;
double deltaError;
double PID1;

double computePID(double input)
{
  currentTime = millis();               // get current time
  elapsedTime = (double)(currentTime - previousTime); // compute time elapsed from previous computation
  error = input - setPoint;
  deltaError = error - lastError;
  double out = kp * error + kd * deltaError;
  lastError = error;      // remember current error
  previousTime = currentTime; // remember current time

  return out; // have function return the PID output
}
//------------------------------------------------------------------------------------------
// establecemos conexion bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup(){
    Serial.begin(9600);
}

void loop(){
if (millis() > tiempo_actual + TICK_ULTRASONIDO)
  {
    tiempo_actual = millis();
    distancia_frontal = sensor_frontal.LeerUltrasonidos();
    distancia_izquierda = sensor_izquierdo.LeerUltrasonidos();
    distancia_derecha = sensor_derecho.LeerUltrasonidos();
    
  }

if (DEBUG_SENSORES)
{
  imprimir_distancia();
}

}