#include <PID.h>
#include <EngineController.h>
#include <DistanceSensors.h>
#include <Button.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//debug
#define TICK_DEBUG 500
unsigned long currentTimePID = 0;
unsigned long currentTimeSensors = 0;

//Motores
#define PIN_RIGHT_ENGINE_IN1 26
#define PIN_RIGHT_ENGINE_IN2 27
#define PIN_LEFT_ENGINE_IN1 18
#define PIN_LEFT_ENGINE_IN2 19
#define PWM_CHANNEL_RIGHT_IN1 1
#define PWM_CHANNEL_RIGHT_IN2 2
#define PWM_CHANNEL_LEFT_IN1 3
#define PWM_CHANNEL_LEFT_IN2 4

//Sharps
#define PIN_SHARP_RIGHT 25
#define PIN_SHARP_LEFT 35
float rightDistance;
float leftDistance;

//veocidades motores pwm
int speedRight = 80;
int speedLeft = 80;
int averageSpeed = 100;

//variables pid
double kp = 1.2;
double kd = 0;
double setPoint;
float BoverPID;
double TICK_PID = 50;

//Boton
#define PIN_BUTTON_START 32
bool start = 0;

IEngine *rightEngine = new Driver_DRV8825(PIN_RIGHT_ENGINE_IN1, PIN_RIGHT_ENGINE_IN2, PWM_CHANNEL_RIGHT_IN1, PWM_CHANNEL_RIGHT_IN2);
IEngine *leftEngine = new Driver_DRV8825(PIN_LEFT_ENGINE_IN1, PIN_LEFT_ENGINE_IN2, PWM_CHANNEL_LEFT_IN1, PWM_CHANNEL_LEFT_IN2);
EngineController *robot = new EngineController(rightEngine, leftEngine);
Isensor *SharpRigh = new Sharp_GP2Y0A21(PIN_SHARP_RIGHT);
Isensor *SharpLeft = new Sharp_GP2Y0A21(PIN_SHARP_LEFT);
Button *buttonStart = new Button(PIN_BUTTON_START);
Pid *PID = new Pid(kp, kd, setPoint, TICK_PID);

void SensorsRead()
{
  rightDistance = SharpRigh->SensorRead();
  leftDistance = SharpLeft->SensorRead();
}

void printPID()
{
  if (millis() > currentTimePID + TICK_DEBUG)
  {
    currentTimePID = millis();
    SerialBT.print("PID: ");
    SerialBT.println(BoverPID);
    SerialBT.print("speedRight: ");
    SerialBT.print(speedRight);
    SerialBT.print(" || speedLeft: ");
    SerialBT.println(speedLeft);
  }
}

void printSensors()
{
  if (millis() > currentTimeSensors + TICK_DEBUG)
  {
    currentTimeSensors = millis();
    SerialBT.print("rightDistance: ");
    SerialBT.print(rightDistance);
    SerialBT.print(" || leftDistance: ");
    SerialBT.println(leftDistance);
  }
}

void setup() {
  SerialBT.begin("Bover");
}

void loop() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    if (command == 'I') 
    {
      start = true;
    } 
    
    else if (command == 'P') 
    {
      start = false;
      robot->Stop(); // Detener los motores cuando se recibe el comando 'P'
    }

    else if (command == 'K') 
    {
      String kpValue = SerialBT.readStringUntil('\n');
      kp = kpValue.toFloat(); // Convertir la cadena a un valor flotante y asignarlo a kd
      SerialBT.print("Kp: ");
      SerialBT.println(kp);
    }
  }

  if(start)
  {
    SensorsRead();
    printSensors();
    float input = rightDistance - leftDistance;
    BoverPID = PID->ComputePid(input);
    printPID();
    if (BoverPID > 30) BoverPID = 30;
    if (BoverPID < -30) BoverPID = -30;
    speedRight = (averageSpeed - (BoverPID));
    speedLeft = (averageSpeed + (BoverPID));
    robot->Forward(speedRight, speedLeft);
  }
  
}
