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


};
//intancio los ultrasonidos
Ultrasonido sensor_frontal = Ultrasonido(TRIG2, ECHO2);
Ultrasonido sensor_derecho = Ultrasonido(TRIG1, ECHO1);
Ultrasonido sensor_izquierdo = Ultrasonido(TRIG4, ECHO4);

void imprimir_distancia(){
    Serial.print("distancia_frontal"  );
    Serial.println(distancia_frontal);
    Serial.print("distancia_izquierda"  );
    Serial.println(distancia_izquierda);
    Serial.print("distancia_derecha"  );
    Serial.println(distancia_derecha);
}

void setup(){
    Serial.begin(9600);
}

void loop(){
if (millis() > tiempo_actual + TICK_ULTRASONIDO)
  {
    tiempo_actual = millis();
    distancia_frontal = sensor_frontal.LeerUltrasonidos(TRIG2, ECHO2);
    distancia_izquierda = sensor_izquierdo.LeerUltrasonidos(TRIG4, ECHO4);
    distancia_derecha = sensor_derecho.LeerUltrasonidos(TRIG1, ECHO1);
    imprimir_distancia();
  }
}