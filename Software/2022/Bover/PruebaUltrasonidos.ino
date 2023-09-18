#include <Ultrasonido.h>
#include "BluetoothSerial.h"

#define PIN_ECCHO_RIGHT 14
#define PIN_TRIGG_RIGHT 12
#define PIN_ECCHO_LEFT 35
#define PIN_TRIGG_LEFT 25
#define PIN_ECCHO_FRONT 32
#define PIN_TRIGG_FRONT 25
float frontDistance;
float rightDistance;
float leftDistance;

unsigned long tiempo_actual = 0;
#define TICK_DEBUG 700
#define DEBUG 1

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

Ultrasonido *ultrasoundRight = new Ultrasonido(PIN_TRIGG_RIGHT, PIN_ECCHO_RIGHT);
Ultrasonido *ultrasoundLeft = new Ultrasonido(PIN_TRIGG_LEFT, PIN_ECCHO_LEFT);
Ultrasonido *ultrasoundFront = new Ultrasonido(PIN_TRIGG_FRONT, PIN_ECCHO_FRONT);


void printSensors()
{
  if (millis() > tiempo_actual + TICK_DEBUG)
  {
    tiempo_actual = millis();
    SerialBT.print("frontDistance: ");
    SerialBT.print(frontDistance);
    SerialBT.print(" || rightDistance: ");
    SerialBT.print(rightDistance);
    SerialBT.print(" || leftDistance: ");
    SerialBT.println(leftDistance);
  }
}

void setup()
{
  SerialBT.begin("BoverOrigin");
  Serial.begin(9600);
}

void loop() 
{
  frontDistance = ultrasoundRight->SensorRead();
  rightDistance = ultrasoundLeft->SensorRead();
  leftDistance = ultrasoundFront->SensorRead();
  printSensors();
}