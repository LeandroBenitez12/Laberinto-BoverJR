#include "PID.h"


Pid::Pid(double p, double d,double sp, double tick)
{
    kp = p;
    kd = d;
    setPoint = sp;
    TICK_PID = tick;
}


double Pid::ComputePid(double inp)
{
    if(millis() > currentTimePID + TICK_PID)
    {
        currentTimePID = millis();
        Input = inp;
        error = Input - setPoint;
        deltaError = error - lastError;
        double out = kp * error + kd * deltaError;
        lastError = error;

        return out;
    }
    
}
