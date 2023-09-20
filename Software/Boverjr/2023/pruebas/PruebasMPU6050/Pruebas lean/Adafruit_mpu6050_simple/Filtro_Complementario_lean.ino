#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

const float alpha = 0.36;  // Coefficient for complementary filter
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float pitch = 0, roll = 0, yaw = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
}

void loop() {
  sensors_event_t accel_event, gyro_event, temp;
  mpu.getEvent(&accel_event, &gyro_event, &temp);

  // Extract accelerometer data
  accX = accel_event.acceleration.x;
  accY = accel_event.acceleration.y;
  accZ = accel_event.acceleration.z;

  // Extract gyroscope data
  gyroX = gyro_event.gyro.x;
  gyroY = gyro_event.gyro.y;
  gyroZ = gyro_event.gyro.z;

  // Calculate pitch, roll, and yaw from accelerometer and gyroscope
  pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  yaw += gyroZ * 0.01; // 0.01 is the time interval in seconds

  // Apply complementary filter to gyroscope and accelerometer data
  pitch = alpha * pitch + (1 - alpha) * gyroX;
  roll = alpha * roll + (1 - alpha) * gyroY;
  yaw = alpha * yaw + (1 - alpha) * gyroZ;

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print(" Yaw: ");
  Serial.print(yaw);
  Serial.print(" Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");

  delay(10);
}