#include <SharpLab.h>
#include "BluetoothSerial.h"

#define PIN_SHARP_RIGH 17
#define PIN_SHARP_LEFT 23
#define PIN_SHARP_FRONT 35
float frontDistance;
float rightDistance;
float leftDistance;

unsigned long tiempo_actual = 0;
#define TICK_DEBUG 500
#define DEBUG 1

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
Button *buttonStart = new Button(PIN_BUTTON_START);

Sharp *SharpFront = new Sharp(PIN_SHARP_FRONT);
Sharp *SharpRigh = new Sharp(PIN_SHARP_RIGH);
Sharp *SharpLeft = new Sharp(PIN_SHARP_LEFT);

void printSensors()
{
    if (millis() > tiempo_actual + TICK_DEBUG)
    {
        tiempo_actual = millis();
        Serial.print("frontDistance: ");
        Serial.print(frontDistance);
        //Serial.print(" || rightDistance: ");
        //Serial.print(rightDistance);
        //Serial.print(" || leftDistance: ");
        //Serial.println(leftDistance);
    }
}

void setup()
{
    SerialBT.begin("Bover");
    Serial.begin(115200);
}

void loop()
{
    frontDistance = SharpFront->SharpDist();
    // rightDistance = SharpRigh->SharpDist();
    // leftDistance = SharpLeft->SharpDist();
    printSensors();
}