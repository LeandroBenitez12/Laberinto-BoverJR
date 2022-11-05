#include <QRE1113.h>

#define SENSOR_FINAL 32

QRE1113 *sensorFinal = new QRE1113(SENSOR_FINAL);
void setup(){
  Serial.begin(9600);
}

void loop(){
  Serial.println(sensorFinal->SensorRead());
}
