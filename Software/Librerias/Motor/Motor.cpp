#include "Motor.h"

Motor::Motor(int pin1, int pin2, int pinpwm, int ch)
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
void Motor::SetVelocidad(int vel)
{
  velocidad = vel;
}
void Motor::Forward()
{
  ledcWrite(channel, velocidad);
  digitalWrite(pin_1, HIGH);
  digitalWrite(pin_2, LOW);
}
void Motor::Backward()
{
  ledcWrite(channel, velocidad);
  digitalWrite(pin_1, LOW);
  digitalWrite(pin_2, HIGH);
}
void Motor::Stop()
{
  ledcWrite(channel, velocidad);
  digitalWrite(pin_1, LOW);
  digitalWrite(pin_2, LOW);
}