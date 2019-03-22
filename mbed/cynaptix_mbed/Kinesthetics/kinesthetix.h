#ifndef KINESTHETIX_H
#define KINESTHETIX_H

#include "mbed.h"
#include "Servo.h"
#include "QEI.h"

Serial pc(USBTX, USBRX);

enum FingerType{
    MIDDLE,
    INDEX,
    THUMB
};

class Kinesthetix{
public:
    // Constructor
    Kinesthetix(FingerType finger);
    // Destructor
    ~Kinesthetix();
    // Control position of servo, call every loop
    void control(float desired);
private:
    Timer t;
    Servo *myservo;
    QEI *wheel;
    int pos;
    float Kp, Ki, Kd;
    float iE, dE;
    volatile float preDesShift, desShift;
    float Kntrl;
    float dt;
};

#endif
