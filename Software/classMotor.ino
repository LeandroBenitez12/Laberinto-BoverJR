// motores
#define ENA 15
#define MR1 18
#define MR2 5
#define ENB 19
#define ML1 4
#define ML2 2
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

    Motor(int pin1, int pin2, int pinpwm, int pwm, int vel, int freq,int bits){
        pin_1 = pin1;
        pin_2 = pin2;
        pin_pwm = pinpwm;
        channel = pwm;
        velocidad = vel;
        freq = freqpwm;
        bits = resolucion;
        
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

//instancio los motores

Motor mDer = Motor(MR1, MR2, ENA, PWMChannel1, velocidad_derecha, freq1, resolution1);
Motor mIzq = Motor(ML1, ML2, ENB, PWMChannel2, velocidad_izquierda, freq2, resolution2);

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

void setup(){
  
  }

void loop(){
  
  }