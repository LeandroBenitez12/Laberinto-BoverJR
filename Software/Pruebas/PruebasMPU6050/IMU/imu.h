#ifndef _IMU_H
#define _IMU_H
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"


class IMU
{
private:
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
    
public:
    IMU(double p, double i, double d, double sp, double tick);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);  // Establece la velocidad de reloj I2C en 400kHz. Comenta esta línea si tienes problemas de compilación
    #endif
    void dmpDataReady();
    double ComputePid(double inp);
};
#endif