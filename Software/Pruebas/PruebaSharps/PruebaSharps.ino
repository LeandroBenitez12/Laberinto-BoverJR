#include <DistanceSensors.h>
#include "BluetoothSerial.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//Sharps
#define PIN_SHARP_RIGHT 35
#define PIN_SHARP_LEFT 27
#define PIN_SHARP_FRONT 17
float rightDistance;
float leftDistance;
float frontDistance;

unsigned long currentTime = 0;
#define TICK_DEBUG 500

Isensor *SharpRigh = new Sharp_GP2Y0A21(PIN_SHARP_RIGHT);
Isensor *SharpLeft = new Sharp_GP2Y0A21(PIN_SHARP_LEFT);
Isensor *SharpFront = new Sharp_GP2Y0A21(PIN_SHARP_FRONT);
void setup() {
  SerialBT.begin("Bover");
  Serial.begin(9600);
}

void loop() {
  rightDistance = SharpRigh->SensorRead();
  leftDistance = SharpLeft->SensorRead();
  frontDistance = SharpFront->SensorRead();

  if (millis() > currentTime + TICK_DEBUG)
  {
    currentTime = millis();
    SerialBT.print("frontDistance: ");
    SerialBT.print(frontDistance);
    SerialBT.print(" || rightDistance: ");
    SerialBT.print(rightDistance);
    SerialBT.print(" || leftDistance: ");
    SerialBT.println(leftDistance);
  }
}
