// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long currenTime = 0;
#define TICK_PRINT 500

long tiempo_prev;
float dt;
float ang_x, ang_y,ang_z;
float ang_x_prev, ang_y_prev,ang_z_prev;
float girosc_ang_x_prev, girosc_ang_y_prev, girosc_ang_z_prev;

void setup() {
  Serial.begin(115200);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor
  //seteamos rango de mediciones
  sensor.setFullScaleAccelRange( (MPU6050_IMU::MPU6050_ACCEL_FS_8));
  sensor.setFullScaleGyroRange( (MPU6050_IMU::MPU6050_GYRO_FS_1000));

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}

void loop() {
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_z = atan2(-ax, ay) * 180 / 3.14;

  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  
  girosc_ang_z_prev=girosc_ang_z;
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;
  ang_z_prev=ang_z;

  //Mostrar los angulos separadas por un [tab]
  if(millis() > currenTime + TICK_PRINT)
  {
    Serial.print("Rotacion en X:  ");
    Serial.print(ang_x); 

    Serial.print("\t Rotacion en Y: ");
    Serial.print(ang_y);
    Serial.print("\t Rotacion en Z: ");
    Serial.println(ang_z);
    
    currenTime = millis();
  }

}