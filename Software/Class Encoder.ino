//
#define PIN_ENCODER 4
int contador = 0;
unsigned long tiempo_actual = 0;
#define TICK 500
float vuelta_completa = 100;


//creo la class del encoder
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

  if(contador>=100)contador=0;
  return contador;

  }
  
//cuento los grados del giro
double Giro(){
  if (pin->DeteccionFlanco()) {
      contador = pin->Contador();
    }
  float giro = contador*360/vuelta_completa;
  return giro;
  }

};

//instancio el encoder
EncoderInflarrojo *Encoder = new EncoderInflarrojo(PIN_ENCODER);






void setup(){
    Serial.begin(9600);

}

void loop(){
  
    float giro = Giro();

    if (millis() > tiempo_actual + TICK)
  {
    tiempo_actual = millis();
    Serial.print("giro de: ");
    Serial.print(giro);
    Serial.println("G");
    
  }
    
}