#include "PID.h"

Pid::Pid(double p, double d, double i, double sp, double tick)
{
    kp = p;
    ki = i;
    kd = d;
    setPoint = sp;
    tick_pid = tick;
}

double Pid::ComputePid(double inp)
{
    if (millis() > currentTime + tick_pid)
    {
        double out;
        currentTime = millis();
        elapsedTime = currentTime - previousTime;
        input = inp;
        error = input - setPoint;
        integral += error* elapsedTime;
        deltaError = (error - lastError)/elapsedTime;

        lastError = error;
        previousTime = currentTime;
        
        return out = kp * error + ki * integral + kd * deltaError;
    }
}
