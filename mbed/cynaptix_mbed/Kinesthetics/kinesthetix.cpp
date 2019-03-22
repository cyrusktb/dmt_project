#include "mbed.h"
#include "Servo.h"
#include "QEI.h"
#include "kinesthetix.h"

Kinesthetix::Kinesthetix(FingerType finger) {
    if (finger == MIDDLE){
        myservo = new Servo(PA_4);
        wheel = new QEI(PA_1, PA_3, NC, 24, QEI::X4_ENCODING);
    }
    else if (finger == INDEX){
        myservo = new Servo(PB_4);
        wheel = new QEI(NC, NC, NC, 24, QEI::X4_ENCODING);
    }
    else if (finger == THUMB){
        myservo = new Servo(PA_11);
        wheel = new QEI(PA_12, PB_0, NC, 24, QEI::X4_ENCODING);
    }

    iE = 0;
    dE = 0;
    Kp = 0.01378;
    Ki = 0.0006875;
    Kd = 0.00011;
}

Kinesthetix::~Kinesthetix() {
    // Free memory
    delete myservo;
    delete wheel;
}

void Kinesthetix::control(float desired) {
        // Time since timer was last reset
        t.start();                      // Start timer
        dt = t.read() * 0.7 + dt * 0.3; // Numerical smoothing as it's quite noisy
        t.reset();                      // Reset timer to 0, but keep counting
        pos = wheel->getPulses();
        desShift = desired - pos; //The "Normalised" Position to stop the motor
        
        //  INTEGRAL
        // Don't integrate error if the position is so far out that kp alone hits max speed
        if(Kp * desShift <= 0.5f && Kp * desShift >= -0.5f) {
            iE += desShift * dt;
            }
            
        //  DERIVATIVE
        preDesShift = desShift;
        dE = (desShift - preDesShift)/dt;
        
        //  THE PULL OF THE TRIGGER
        Kntrl = Kp * desShift + Ki * iE + Kd * dE; //Proportional Integral Derivative Control
        *myservo = 0.5 + Kntrl;
}     
