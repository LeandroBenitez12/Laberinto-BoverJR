#include <SharpLab.h>



//sensor de distancia
#define PIN_SENSOR_DISTANCIA_DERECHO 4
#define RIVAL 30
float distSharpRigh;

unsigned long tiempo_actual = 0;
#define TICK_DEBUG 500
#define DEBUG 1

Sharp *sharpRight = new Sharp(PIN_SENSOR_DISTANCIA_DERECHO);

void setup()
{
    Serial.begin(9600);
}

void loop() 
{
    distSharpRigh = sharpRight->SharpDist();

    if(DEBUG)
    {
        if (millis() > tiempo_actual + TICK_DEBUG)
        {
            Serial.print("dist: ");
            Serial.println(distSharpRigh);

        }

    }
  
}