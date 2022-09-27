/*#define PIN_BUTTON_START 39
#define PIN_LED 2
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
  void setPrenderLed()
  {
    digitalWrite(pin, HIGH);
  }
  void setApagarLed()
  {
    digitalWrite(pin, LOW);
  }
};
class Button
{
private:
  int pin;

  // metodo
public:
  Button(int p)
  {
    pin = p;

    pinMode(pin, INPUT);
  }

  // metodos o acciones
  bool getIsPress()
  {
    bool estado = digitalRead(pin);
    return estado;
  }
};

// Instancio los buttons *punteros
Button *start = new Button(PIN_BUTTON_START);
// Instancio los buzzers
Buzzer *Led = new Buzzer(PIN_LED);

// buzzers
void LedOn()
{
  Led->setPrenderLed();
}
void LedOff()
{
  Led->setApagarLed();
}
void setup()
{
   Serial.begin(9600); 
}

void loop()
{
bool boton_start = start->getIsPress();

if (boton_start)
{
    LedOn();
    Serial.print("si");
}
else
{
    LedOff();
    Serial.print("no");
}
Serial.print(boton_start);
}*/
