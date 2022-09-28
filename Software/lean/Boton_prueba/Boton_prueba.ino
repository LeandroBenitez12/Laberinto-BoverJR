// button
#define BUTTON 39
bool button_start;

int LeerButton(int button)
{
  bool estado_button;
  estado_button = digitalRead(button);
  return estado_button;
}
void setup(){
  Serial.begin(115200);
    pinMode(BUTTON, INPUT);
}
  void loop(){
  button_start = LeerButton(BUTTON);
  Serial.print(button_start);
  Serial.println(" | ");
  }
