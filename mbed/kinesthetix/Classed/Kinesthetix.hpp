#ifndef KINESTHETIX_H
#define KINESTHETIX_H

#include "mbed.h"
#include "Servo.h"
#include "QEI.h"

enum FingerType{
    MIDDLE,
    INDEX,
    THUMB
};

class Kinesthetix{
public:
    // Constructor
    Kinesthetix(FingerType finger);
    // Control position of servo, call every loop
    void control(float desired);
private:
    Timer t;
    Servo myservo(char pinS);
    QEI wheel(char pin1, char pin2, char NC, float Pul, char Type);
    int pos;
    float Kp, Ki, Kd;
    float iE, dE;
    volatile int desired;
    volatile float preDesShift, desShift;
    float Kntrl;
    float dt;
};

#endif