// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

unsigned long currenTime = 0;
#define TICK_PRINT 500
// Valores RAW (sin procesar) del acelerometro  en los ejes x,y,z
int16_t ax, ay, az;

void setup() {
  Serial.begin(115200);  //Iniciando puerto serial
  Wire.begin();          //Iniciando I2C
  sensor.initialize();   //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}

void loop() {
  // Leer las aceleraciones
  sensor.getAcceleration(&ax, &ay, &az);
  //Calcular los angulos de inclinacion:
  float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  //Mostrar los angulos separadas por un [tab]
 if (millis() > currenTime + TICK_PRINT) {
    Serial.print("Inclinacion en X: ");
    Serial.print(accel_ang_x);
    Serial.print("  //  ");
    Serial.print("tInclinacion en Y:");
    Serial.println(accel_ang_y);
    currenTime = millis();
  }
<<<<<<< HEAD

}
=======
  
}
>>>>>>> 67d4ecb29658af519c34555a779af10f2f575977
