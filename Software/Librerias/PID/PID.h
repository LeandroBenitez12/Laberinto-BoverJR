#ifndef _PID_H
#define _PID_H
#include "Arduino.h"

class Pid
{
private:
    double kp = 0;
    double kd = 0;
    double ki = 0;
    double error;
    double lastError;
    double Input = 0;
    double output = 0;
    double setPoint = 0;
    double deltaError;
    double integral;
    unsigned long currentTimePID = 0;
    int TICK_PID = 70;

public:
    Pid(double p, double d, double i, double sp, double tick);
    double ComputePid(double inp);
};
#endif