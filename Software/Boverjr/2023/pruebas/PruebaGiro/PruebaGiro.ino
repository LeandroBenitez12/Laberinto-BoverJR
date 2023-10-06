// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "BluetoothSerial.h"
#include <EngineController.h>

//Motores
#define PIN_RIGHT_ENGINE_IN1 26
#define PIN_RIGHT_ENGINE_IN2 27
#define PIN_LEFT_ENGINE_IN1 18
#define PIN_LEFT_ENGINE_IN2 19
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 mpu;


IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *Bover = new EngineController(rightEngine, leftEngine);

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int16_t gx, gy, gz;

unsigned long currenTime = 0;
#define TICK_PRINT_Z 500

long tiempo_prev, dt;
float girosc_ang_x, girosc_ang_y, girosc_ang_z;
float girosc_ang_x_prev, girosc_ang_y_prev, girosc_ang_z_prev;

void printReadZ(){
  if(millis() > currenTime + TICK_PRINT_Z)
  {
    //Mostrar los angulos 
    SerialBT.print(" Rotacion en z: ");
    SerialBT.println(girosc_ang_z);
    currenTime = millis();
  }
}

void calculoAnguloZ(){
  // Leer las velocidades angulares 
  mpu.getRotation(&gx, &gy, &gz);

  //Calcular los angulos rotacion:
  dt = millis()-tiempo_prev;
  tiempo_prev=millis();
  
  girosc_ang_z = (gz/131)*dt/1000.0 + girosc_ang_z_prev;

  girosc_ang_z_prev=girosc_ang_z;

}

float turnRight() {
  Bover->Right(255,255);
  SerialBT.println("RIGHT");
  if (girosc_ang_z > 90){
    Bover->Forward(255,255);
    SerialBT.println("FORWARD");
    delay(3000);
    Bover->Stop();
  }
}

void setup() {
  Serial.begin(115200);    //Iniciando puerto serial
  SerialBT.begin("Bover JR");
  Wire.begin();           //Iniciando I2C  
  mpu.initialize();    //Iniciando el mpu

  if (mpu.testConnection()) Serial.println("mpu iniciada correctamente");
  else Serial.println("Error al iniciar la mpu");
  tiempo_prev=millis();
}

void loop() {

  calculoAnguloZ();
  printReadZ();
  if (SerialBT.available()) {
    
    Serial.print("INICIAR? ");
    bool stateStart = SerialBT.read();
      if(stateStart == true) {
        turnRight();
      }
  
}
}