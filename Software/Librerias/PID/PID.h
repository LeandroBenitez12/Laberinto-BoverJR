#ifndef _PID_H
#define _PID_H
#include "Arduino.h"

class Pid
{
private:
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double error;
    double lastError;
    double input = 0;
    double output = 0;
    double setPoint = 0;
    double deltaError;
    double integral;
    unsigned long currentTime = 0;
    unsigned long elapsedTime = 0;
    unsigned long previousTime = 0;
    int tick_pid = 20;

public:
    Pid(double p, double i, double d, double sp, double tick);
    double ComputePid(double inp);
};
#endif