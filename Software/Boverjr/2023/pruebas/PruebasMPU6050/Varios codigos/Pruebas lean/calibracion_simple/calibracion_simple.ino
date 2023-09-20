#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <EEPROM.h>

Adafruit_MPU6050 mpu;

// Valores de offset para cada eje
float accX_offset = 0.04;
float accY_offset = -0.02;
float accZ_offset = 10.26;
float gyroX_offset = 0.02;
float gyroY_offset = -0.01;
float gyroZ_offset = -0.00;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Configura el rango del acelerómetro
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Configura el rango del giroscopio
}

void loop() {
  // Variables para almacenar los valores de los sensores
  float accX, accY, accZ, gyroX, gyroY, gyroZ;
  
  // Lee los valores de los sensores
  sensors_event_t accel_event, gyro_event, temp_event;
  mpu.getEvent(&accel_event, &gyro_event, &temp_event);
  accX = accel_event.acceleration.x - accX_offset;
  accY = accel_event.acceleration.y - accY_offset;
  accZ = accel_event.acceleration.z - accZ_offset;
  gyroX = gyro_event.gyro.x - gyroX_offset;
  gyroY = gyro_event.gyro.y - gyroY_offset;
  gyroZ = gyro_event.gyro.z - gyroZ_offset;
  
  // Muestra los valores de los sensores
  Serial.print("Acelerómetro: ");
  Serial.print(accX);
  Serial.print(", ");
  Serial.print(accY);
  Serial.print(", ");
  Serial.print(accZ);
  Serial.print("   ");
  Serial.print("Giroscopio: ");
  Serial.print(gyroX);
  Serial.print(", ");
  Serial.print(gyroY);
  Serial.print(", ");
  Serial.println(gyroZ);

  // Espera antes de repetir la lectura
  delay(100);
}
