#include <Encoder.h>
#include "BluetoothSerial.h"

#define PIN_ENCODER_RIGHT 13
#define PIN_ENCODER_LEFT 23
int contRight;
int contLeft;

unsigned long tiempo_actual = 0;
#define TICK_DEBUG 700
#define DEBUG 1

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

Encoder *encoderRight = new Encoder(PIN_ENCODER_RIGHT);
Encoder *encoderLeft = new Encoder(PIN_ENCODER_LEFT);

void printCont()
{
  if (millis() > tiempo_actual + TICK_DEBUG)
  {
    tiempo_actual = millis();
    SerialBT.print("contRight: ");
    SerialBT.print(contRight);
    SerialBT.print(" // contLeft: ");
    SerialBT.println(contLeft);
  }
}

void setup()
{
  SerialBT.begin("BoverOrigin");
  Serial.begin(9600);
}

void loop() 
{
    contRight = encoderRight->Cont();
    contLeft = encoderLeft->Cont();
    printCont();
}