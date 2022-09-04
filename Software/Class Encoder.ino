//encoder
#define PIN_ENCODER_IZQ 1
#define PIN_ENCODER_DER 2
bool lecturaEncoderIzq;
bool lecturaEncoderDer;
bool lecturaEncoder;

//creo la class del encoder
class EncoderInflarrojo{

private:
    int pin;
    bool flanco = HIGH;
    bool estado_anterior = !flanco;

public:

     EncoderInflarrojo(int p){
        
        p = pin;
        pinMode(pin,INPUT);
    }

//funcion para la deteccion del cambio de flanco
    bool DeteccionFlanco() {
      bool estado_actual = digitalRead(pin);
      bool estado = (estado_anterior != estado_actual) && estado_actual == flanco;
      estado_anterior = estado_actual;
      return estado;
    }
};

//instancio el encoder
EncoderInflarrojo EncoderDer = EncoderInflarrojo(PIN_ENCODER_IZQ);
EncoderInflarrojo EncoderIzq = EncoderInflarrojo(PIN_ENCODER_DER);




void setup(){

}

void loop(){
    int cambioFlancoDer = EncoderDer.DeteccionFlanco();
    int cambioFlancoIzq = EncoderIzq.DeteccionFlanco();
}