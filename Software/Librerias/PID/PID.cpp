#include "PID.h"

Pid::Pid(double p, double d, double i, double sp, double tick)
{
    kp = p;
    kd = d;
    ki = i;
    setPoint = sp;
    TICK_PID = tick;
}

double Pid::ComputePid(double inp)
{
    if (millis() > currentTimePID + TICK_PID)
    {
        double out;
        currentTimePID = millis();
        Input = inp;
        error = Input - setPoint;
        deltaError = error - lastError;
        lastError = error;
        integral += error;

        return out = kp * error + kd * deltaError + ki * integral;
    }
}
