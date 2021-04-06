#include <PID_v1.h>

#ifndef MOTOR_H
#define MOTOR_H


#define clockwise       0x1
#define cClockwise      0x0

class Motor
{
public:
    PID *pid; 
    int maxTicks; 
    int minTicks;
    int motorPin1;
    int motorPin2;
    int encoderPin1;
    int encoderPin2;

    Motor(PID * pid, int max, int min, int p1, int p2, int p3, int p4);

    void setMotor(int power, int direction, int currentTick);

};

#endif

 