#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
public:
    int maxTicks; 
    int minTicks;
    int motorPin1;
    int motorPin2;
    int encoderPin1;
    int encoderPin2;

    Motor(int max, int min, int p1, int p2, int p3, int p4);

    void setMotor(int power, int direction);

};

#endif

 