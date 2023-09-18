#include <Button.h>
#include "BluetoothSerial.h"

#define PIN_BUTTON_START 32



#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

void setup() {
  SerialBT.begin("Bover");
}

void loop() {
  if (buttonStart->GetIsPress()) SerialBT.println("Button start is press");
  else SerialBT.println("----------------");
}
