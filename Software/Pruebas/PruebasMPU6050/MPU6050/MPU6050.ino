
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <EngineController.h>

// Motores
#define PIN_RIGHT_ENGINE_IN1 26
#define PIN_RIGHT_ENGINE_IN2 27
#define PIN_LEFT_ENGINE_IN1 19
#define PIN_LEFT_ENGINE_IN2 18
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4
#define INTERRUPT_PIN 4
#define LED_PIN 2

bool blinkState = false;

// Declaración de variables para el manejo del MPU
bool dmpReady = false;   // Se establece en verdadero si la inicialización de DMP tiene éxito
uint8_t mpuIntStatus;    // Almacena el estado real de la interrupción del MPU
uint8_t devStatus;       // Almacena el estado de retorno después de cada operación del dispositivo (0 = éxito, !0 = error)
uint16_t packetSize;     // Tamaño esperado del paquete DMP (predeterminado es 42 bytes)
uint16_t fifoCount;      // Cuenta de todos los bytes actualmente en el FIFO
uint8_t fifoBuffer[64];  // Búfer de almacenamiento FIFO

Quaternion q;         // Quaternion para almacenar la orientación (w, x, y, z)
VectorInt16 aa;       // Vector de aceleración bruta (x, y, z)
VectorInt16 aaReal;   // Vector de aceleración corregida (x, y, z)
VectorInt16 aaWorld;  // Vector de aceleración en el mundo (x, y, z)
VectorFloat gravity;  // Vector de gravedad (x, y, z)
float ypr[3];         // Matriz de ángulos Yaw, Pitch y Roll (yaw, pitch, roll)
float gyroZ;
volatile bool mpuInterrupt = false;

IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *robot = new EngineController(rightEngine, leftEngine);
MPU6050 mpu;

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
// Iniciar el bus I2C
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // Establece la velocidad de reloj I2C en 400kHz. Comenta esta línea si tienes problemas de compilación
#endif

  // Iniciar la comunicación serial
  Serial.begin(9600);

  // Inicializar el MPU6050
  Serial.println(F("Inicializando dispositivos I2C..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // Comprobar la conexión
  Serial.println(F("Probando conexiones de dispositivos..."));
  Serial.println(mpu.testConnection() ? F("Conexión MPU6050 exitosa") : F("Conexión MPU6050 fallida"));

  // Inicializar DMP
  Serial.println(F("Inicializando DMP..."));
  devStatus = mpu.dmpInitialize();

  // Valores de calibración (ajusta estos valores según tu propia calibración)
  mpu.setXGyroOffset(53);
  mpu.setYGyroOffset(47);
  mpu.setZGyroOffset(-65);
  mpu.setZAccelOffset(1071);

  // Activar DMP
  if (devStatus == 0) {
    Serial.println(F("Habilitando DMP..."));
    mpu.setDMPEnabled(true);

    // Activar interrupción
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("¡DMP listo! Esperando la primera interrupción..."));
    dmpReady = true;

    // Obtener el tamaño esperado del paquete DMP para la comparación posterior
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = carga inicial de memoria fallida
    // 2 = actualizaciones de configuración DMP fallidas
    // (si algo va a fallar, generalmente será el código 1)
    Serial.print(F("Fallo en la inicialización del DMP (código "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  // Si la inicialización DMP falló, detener el programa
  if (!dmpReady) return;

  // Ejecutar mientras no haya interrupción
  while (!mpuInterrupt && fifoCount < packetSize) {
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Obtener datos del FIFO
  fifoCount = mpu.getFIFOCount();

  // Controlar el desbordamiento
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("¡Desbordamiento del FIFO!"));
  } else if (mpuIntStatus & 0x02) {
    // Esperar la longitud de datos disponibles correcta, debería ser una espera MUY corta
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // Leer un paquete del FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Realizar cálculos de orientación (Yaw, Pitch, Roll) y mostrarlos
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    gyroZ = ypr[0] * 180 / M_PI;
    Serial.print("giroz: /t");
    Serial.println(gyroZ);

    delay(1);
  }
  if (gyroZ >= 0.0 && gyroZ < 90.0) {
    robot->Forward(255, 255);
  } else if (gyroZ <= 0.0 && gyroZ > -90.0) {
    robot->Backward(100, 100);
  } else if (gyroZ <= -90 && gyroZ > -179.9) {
    robot->Backward(255, 255);
  } else if (gyroZ >= 90 && gyroZ <= 179.9) {
    robot->Backward(255, 255);
  }
}
