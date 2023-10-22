#ifndef _PID_H
#define _PID_H
#include "Arduino.h"

class Pid
{
private:
    double kp = 0;
    double kd = 0;
<<<<<<< HEAD
    double ki = 0;
=======
    double ki= 0;
>>>>>>> d0b76f3ee8112b5f2127c17d0ac5584b443cd0dd
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
<<<<<<< HEAD
    Pid(double p, double d, double i, double sp, double tick);
=======
    Pid(double p, double d,double i , double sp, double tick);
>>>>>>> d0b76f3ee8112b5f2127c17d0ac5584b443cd0dd
    double ComputePid(double inp);
};
#endif