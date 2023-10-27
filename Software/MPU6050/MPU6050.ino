#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <EngineController.h>
#include "BluetoothSerial.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// Motores
#define PIN_RIGHT_ENGINE_IN1 27
#define PIN_RIGHT_ENGINE_IN2 26
#define PIN_LEFT_ENGINE_IN1 18
#define PIN_LEFT_ENGINE_IN2 19
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4
#define INTERRUPT_PIN 4
#define LED_PIN 2
#define TICK_DEBUG_Z 500
float gyroZ;
unsigned long currentTimeDer;
unsigned long currentTimeStop;
unsigned long currentTimeZ;
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68
// AD0 high = 0x69

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *robot = new EngineController(rightEngine, leftEngine);
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;        // [w, x, y, z]
VectorInt16 aa;      // [x, y, z]
VectorInt16 aaReal;  // [x, y, z]
VectorInt16 aaWorld; // [x, y, z]
VectorFloat gravity; // [x, y, z]
float ypr[3];        // [yaw, pitch, roll]

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void mpuSetup()
{

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);
  SerialBT.begin("Bover");
  // Iniciar MPU6050
  SerialBT.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // Comprobar  conexion
  SerialBT.println(F("Testing device connections..."));
  SerialBT.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Iniciar DMP
  SerialBT.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Valores de calibracion
  mpu.setXGyroOffset(53);
  mpu.setYGyroOffset(57);
  mpu.setZGyroOffset(-61);
  mpu.setZAccelOffset(1063);

  // Activar DMP
  if (devStatus == 0)
  {
    SerialBT.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Activar interrupcion
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    SerialBT.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    SerialBT.print(F("DMP Initialization failed (code "));
    SerialBT.print(devStatus);
    SerialBT.println(F(")"));
  }
}
void mpuLoop()
{
  // Si fallo al iniciar, parar programa
  if (!dmpReady)
    return;

  // Ejecutar mientras no hay interrupcion
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // AQUI EL RESTO DEL CODIGO DE TU PROGRRAMA
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Obtener datos del FIFO
  fifoCount = mpu.getFIFOCount();

  // Controlar overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 4096)
  {
    mpu.resetFIFO();
    SerialBT.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // MMostrar Yaw, Pitch, Roll
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    gyroZ = ypr[0] * 180 / M_PI;
    if (millis() > currentTimeZ + TICK_DEBUG_Z)
    {
      currentTimeZ = millis();
      SerialBT.print("Eje Z");
      SerialBT.print(gyroZ);
    }
    delay(1);
  }
  void setup()
  {
    mpuSetup();
  }

  void loop()
  {

    mpuLoop();
    // Controlar el giro del robot hacia 90 grados
    /*if (gyroZ > 105) {
        // Girar a la izquierda
        SerialBT.println("HACIA LA IZQUIERDA  ");
        robot->Left(220, 220);
    } else if (gyroZ < 75) {
        // Girar a la derecha
        SerialBT.println("HACIA LA DERECHA  ");
        robot->Right(220, 220);
    } else {
        // Detener el robot cuando alcanza aproximadamente 90 grados
        robot->Stop();
        SerialBT.println("DETENIDO");
    }
    */
    if (gyroZ >= 0 && gyroZ <= 180)
    {
      do
      {
        robot->Right(160, 160);
        if (millis() > currentTimeDer + TICK_DEBUG_Z)
        {
          currentTimeDer = millis();
          SerialBT.println("HACIA LA DERECHA  ");
        }
      } while (gyroZ > -180 && gyroZ < 0);
    }
    else
    {
      robot->Stop();
      if (millis() > currentTimeStop + TICK_DEBUG_Z)
      {
        currentTimeStop = millis();
        SerialBT.println("detenido  ");
      }
    }
  }
}
