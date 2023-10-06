#include <Button.h>
#include "BluetoothSerial.h"

#define PIN_BUTTON_START 32

unsigned long tiempo_actual = 0;
#define TICK_DEBUG 1500
#define DEBUG 1

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
Button *buttonStart1 = new Button(PIN_BUTTON_START);

void printSensors()
{
    if (millis() > tiempo_actual + TICK_DEBUG)
    {
        tiempo_actual = millis();
        bool stateStart = buttonStart1->GetIsPress();
        Serial.print("Dato recibido: ");
        if(stateStart == true) {
            Serial.println(" Press");
        }
        else {
            Serial.println("No");
        }
    }
}

void setup()
{
    SerialBT.begin("Bover");
    Serial.begin(115200);
}

void loop()
{
    printSensors();
}
        // Aquí puedes agregar la lógica para procesar los datos recibidos.
    
  