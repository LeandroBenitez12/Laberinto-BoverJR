// libreries
#include <PID.h>
#include <EngineController.h>
#include <DistanceSensors.h>
#include <Button.h>
#include "BluetoothSerial.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// menu
bool menusalir = false;
bool wall = false;

// mpu
#define INTERRUPT_PIN 4
#define TICK_DEBUG_Z 500
float gyroZ;
unsigned long currentTimeDer;
unsigned long currentTimeStop;
unsigned long currentTimeZ;

// debug
#define TICK_DEBUG_ALL 1000
#define DEBUG_BUTTON 1
#define DEBUG_STATUS 1
#define DEBUG_SENSORS 1
#define DEBUG_PID 1
unsigned long currentTimePID = 0;
unsigned long currentTimeDebugAll = 0;
unsigned long currentTimeMenu = 0;
#define TICK_MENU 5000

// Motores
#define PIN_RIGHT_ENGINE_IN1 27
#define PIN_RIGHT_ENGINE_IN2 26
#define PIN_LEFT_ENGINE_IN1 18
#define PIN_LEFT_ENGINE_IN2 19
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4

// Sharps
#define PIN_SHARP_RIGHT 25
#define PIN_SHARP_LEFT 35
#define PIN_SHARP_FRONT 33
float rightDistance;
float leftDistance;
float frontDistance;
#define PARED_ENFRENTE 10
#define PARED_COSTADO_PASILLO 26
#define NO_HAY_PARED 28
#define NO_HAY_PARED_ENFRENTE 16

// veocidades motores pwm
#define VELOCIDAD_GIROS_90 255
int tick_giro_90 = 200;
int tick_giro_180 = 290;
#define ENTRAR_EN_PASILLO 200
#define DELAY_TOMAR_DECISION 100
#define DELAY_ANTI_INERCIA 20

#define MAX_VEL 255
int speedRightPID;
int speedLeftPID;
int averageSpeedRight = 120;
int averageSpeedLeft = 130;

// variables pid
double kp = 0.90;
double kd = 0.70;
double ki = 0.0;
double setPoint;
double gananciaPID;
double TICK_PID = 1;

// Boton
#define PIN_BUTTON_START 32
bool stateStartButton = 0;

IEngine *leftEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *rightEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *Bover = new EngineController(rightEngine, leftEngine);
Isensor *SharpRigh = new Sharp_GP2Y0A21(PIN_SHARP_RIGHT);
Isensor *SharpLeft = new Sharp_GP2Y0A21(PIN_SHARP_LEFT);
Isensor *SharpFront = new Sharp_GP2Y0A21(PIN_SHARP_FRONT);
Pid *PID = new Pid(kp, kd, ki, setPoint, TICK_PID);
Button *buttonStart1 = new Button(PIN_BUTTON_START);
MPU6050 mpu;
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
}
  void SensorsRead()
  {
    frontDistance = SharpFront->SensorRead();
    rightDistance = SharpRigh->SensorRead();
    leftDistance = SharpLeft->SensorRead();
  }

  void printButton()
  {
    SerialBT.print("Button Start: ");
    SerialBT.println(stateStartButton);
  }
  void printPID()
  {
    if (millis() > currentTimePID + TICK_DEBUG_ALL)
    {
      currentTimePID = millis();
      SerialBT.println("");
      SerialBT.print("Ganancia PID: ");
      SerialBT.println(gananciaPID);
      SerialBT.print("speedRight: ");
      SerialBT.print(speedRightPID);
      SerialBT.print(" || speedLeft: ");
      SerialBT.println(speedLeftPID);
      SerialBT.println("");
    }
  }

  void printSensors()
  {
    SerialBT.print("LeftDistance: ");
    SerialBT.println(leftDistance);
    SerialBT.print("frontDistance: ");
    SerialBT.println(frontDistance);
    SerialBT.print(" || rightDistance: ");
    SerialBT.println(rightDistance);
  }
  enum movement
  {
    STANDBY,
    CONTINUE,
    STOP,
    RIGHT_TURN,
    LEFT_TURN,
    FULL_TURN,
    POST_TURN,
  };
  int movement = STANDBY;

  void turnRight()
  {
    if (gyroZ >= -5 && gyroZ <= 5)
    {
      do
      {
        Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
        if (millis() > currentTimeDer + TICK_DEBUG_Z)
        {
          currentTimeDer = millis();
          SerialBT.println("HACIA LA DERECHA  ");
        }
      } while (gyroZ >= 85 && gyroZ <= 95);
    }
    else
    {
      SerialBT.println("detenido  ");
      movement = POST_TURN;
      /*if (millis() > currentTimeStop + TICK_DEBUG_Z)
      {
        currentTimeStop = millis();
        SerialBT.println("detenido  ");
      }*/
    }  
  }

  void turnLeft()
  {
    if (gyroZ >= -5 && gyroZ <= 5)
    {
      do
      {
        Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
        if (millis() > currentTimeDer + TICK_DEBUG_Z)
        {
          currentTimeDer = millis();
          SerialBT.println("HACIA LA DERECHA  ");
        }
      } while (gyroZ <= -85 && gyroZ >= -95);
    }
    else
    {
      SerialBT.println("detenido  ");
      movement = POST_TURN;
      /*if (millis() > currentTimeStop + TICK_DEBUG_Z)
      {
        currentTimeStop = millis();
        SerialBT.println("detenido  ");
      }*/
    }  
  }

  void fullTurn()
  {
    if (gyroZ >= -5 && gyroZ <= 5)
    {
      do
      {
        Bover->Right(VELOCIDAD_GIROS_90, VELOCIDAD_GIROS_90);
        if (millis() > currentTimeDer + TICK_DEBUG_Z)
        {
          currentTimeDer = millis();
          SerialBT.println("HACIA LA DERECHA  ");
        }
      } while (gyroZ >= 172 && gyroZ <= 180 || gyroZ <= -172 && gyroZ >= -180);
    }
    else
    {
      SerialBT.println("detenido  ");
      movement = POST_TURN;
      /*if (millis() > currentTimeStop + TICK_DEBUG_Z)
      {
        currentTimeStop = millis();
        SerialBT.println("detenido  ");
      }*/
    }  
  }

  void postTurn()
  {
    Bover->Forward(averageSpeedRight, averageSpeedLeft);
    delay(ENTRAR_EN_PASILLO);
  }

  void processCommand(char c)
  {
    switch (c)
    {
    case '1':
      SerialBT.println("Configurando averageSpeedRight, ingresa un valor(0-255): ");
      while (SerialBT.available() == 0)
        ;
      averageSpeedRight = SerialBT.parseInt();
      Serial.print("Velocidad Derecha =  ");
      Serial.println(averageSpeedRight);
      delay(500);
      break;
    case '2':
      SerialBT.println("Configurando averageSpeedLeft, ingresa un valor(0-255): ");
      while (SerialBT.available() == 0)
        ;
      averageSpeedLeft = SerialBT.parseInt();
      Serial.print("Velocidad Izquierda =  ");
      Serial.println(averageSpeedLeft);
      delay(500);
      break;
    case '3':
      SerialBT.println("Configurando kp 0.0: ");
      while (SerialBT.available() == 0)
        ;
      kp = SerialBT.parseFloat();
      Serial.print("KP =  ");
      Serial.println(kp);
      delay(500);
      break;
    case '4':
      SerialBT.println("Configurando kd 0.0: ");
      while (SerialBT.available() == 0)
        ;
      kd = SerialBT.parseFloat();
      Serial.print("KD =  ");
      Serial.println(kd);
      delay(500);
      break;
    case '5':
      while (SerialBT.available() == 0)
        ;
      wall = true;
      SerialBT.println("Modo pared Derecha activado.");
      delay(500);
      break;
    case '6':
      while (SerialBT.available() == 0)
        ;
      menusalir = true;
      SerialBT.println("Saliendo.");
      delay(500);
      break;
    default:
      SerialBT.println("Comando no válido.");
      break;
    }
  }

  void movementLogic()
  {
    switch (movement)
    {
    case STANDBY:
    {
      Bover->Stop();
      if (stateStartButton)
      {
        delay(2000);
        movement = CONTINUE;
      }
    }
    break;
    case CONTINUE:
    {
      float input = rightDistance - leftDistance;
      gananciaPID = PID->ComputePid(input);
      if (DEBUG_PID)
        printPID();

      speedRightPID = (averageSpeedRight - (gananciaPID));
      speedLeftPID = (averageSpeedLeft + (gananciaPID));
      if (speedLeftPID >= MAX_VEL)
        speedLeftPID = MAX_VEL;
      if (speedRightPID >= MAX_VEL)
        speedRightPID = MAX_VEL;

      Bover->Forward(speedRightPID, speedLeftPID);

      if (frontDistance <= PARED_ENFRENTE)
        movement = STOP;
      if (wall == true)
      {
        if (leftDistance > NO_HAY_PARED && frontDistance > NO_HAY_PARED_ENFRENTE && rightDistance > NO_HAY_PARED)
        {
          movement = RIGHT_TURN;
        }
        if (leftDistance < PARED_COSTADO_PASILLO && frontDistance > NO_HAY_PARED_ENFRENTE && rightDistance > NO_HAY_PARED)
        {
          movement = RIGHT_TURN;
        }
        if (leftDistance > PARED_COSTADO_PASILLO && frontDistance > NO_HAY_PARED_ENFRENTE && rightDistance < NO_HAY_PARED)
        {
          movement = POST_TURN;
        }
      }
      else
      {
        if (leftDistance > NO_HAY_PARED && frontDistance > NO_HAY_PARED_ENFRENTE && rightDistance > NO_HAY_PARED)
        {
          movement = LEFT_TURN;
        }
        if (leftDistance > PARED_COSTADO_PASILLO && frontDistance > NO_HAY_PARED_ENFRENTE && rightDistance < NO_HAY_PARED)
        {
          movement = LEFT_TURN;
        }
        if (leftDistance < PARED_COSTADO_PASILLO && frontDistance > NO_HAY_PARED_ENFRENTE && rightDistance > NO_HAY_PARED)
        {
          movement = POST_TURN;
        }
      }
      break;
    }
    case STOP:
    {
      Bover->Stop();
      delay(DELAY_TOMAR_DECISION);
      if (frontDistance <= PARED_ENFRENTE && rightDistance > NO_HAY_PARED && leftDistance <= PARED_COSTADO_PASILLO)
        movement = RIGHT_TURN;
      if (frontDistance <= PARED_ENFRENTE && rightDistance <= PARED_COSTADO_PASILLO && leftDistance > NO_HAY_PARED)
        movement = LEFT_TURN;
      if (frontDistance <= PARED_ENFRENTE && rightDistance > NO_HAY_PARED && leftDistance > NO_HAY_PARED)
      {
        if (wall = true)
        {
          movement = RIGHT_TURN;
        }
        else
        {
          movement = LEFT_TURN;
        }
      }
      if (frontDistance <= PARED_ENFRENTE && rightDistance <= PARED_COSTADO_PASILLO && leftDistance <= PARED_COSTADO_PASILLO)
        movement = FULL_TURN;

      else if (frontDistance > PARED_ENFRENTE)
        movement = CONTINUE;
      break;
    }

    case RIGHT_TURN:
    {
      turnRight();
      break;
    }

    case LEFT_TURN:
    {
      turnLeft();
      Bover->Stop();
      delay(DELAY_ANTI_INERCIA);
      movement = POST_TURN;
      break;
    }

    case FULL_TURN:
    {
      fullTurn();
      Bover->Stop();
      delay(DELAY_ANTI_INERCIA);
      movement = POST_TURN;
      break;
    }

    case POST_TURN:
    {
      postTurn();
      movement = CONTINUE;
      break;
    }
    }
  }

  void printStatus()
  {
    String state = "";
    switch (movement)
    {
    case STANDBY:
      state = "STANDBY";
      break;
    case CONTINUE:
      state = "CONTINUE";
      break;
    case STOP:
      state = state = "STOP";
      break;
    case RIGHT_TURN:
      state = "RIGHT TURN";
      break;
    case LEFT_TURN:
      state = "LEFT TURN";
      break;
    case FULL_TURN:
      state = "FULL TURN";
      break;
    case POST_TURN:
      state = "POST TURN";
      break;
    }
    SerialBT.print("State: ");
    SerialBT.println(state);
  }

  void printAll()
  {
    if (millis() > currentTimeDebugAll + TICK_DEBUG_ALL)
    {
      currentTimeDebugAll = millis();
      if (DEBUG_BUTTON)
        printButton();
      SerialBT.println("");
      if (DEBUG_SENSORS)
        printSensors();
      SerialBT.println("");
      if (DEBUG_STATUS)
        printStatus();
      SerialBT.println("");
    }
  }
  void setup()
  {
    Serial.begin(9600);
    SerialBT.begin("Bover");
    mpuSetup();
  }

  void loop()
  {
    mpuLoop();
    stateStartButton = buttonStart1->GetIsPress();
    SensorsRead();
    if (SerialBT.available())
    {
      char command = SerialBT.read();
      processCommand(command);
    }
    if (stateStartButton == true)
      menusalir = true;
    if (menusalir == true)
    {
      movementLogic();
      printAll();
    }
  }