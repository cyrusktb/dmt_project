#include "mbed.h"
#include "Servo.h"
#include "QEI.h"
#include "Kinesthetix.hpp"

Kinesthetix::Kinesthetix(FingerType finger) :myservo(NC), wheel(NC, NC, NC, 24){
    if (finger == MIDDLE){
        myservo = Servo(PA_4);
        wheel = QEI(PA_1, PA_3, NC, 24, QEI::X4_ENCODING);
    };
    if (finger == INDEX){
        myservo = Servo(PB_4);
    };
    if (finger == THUMB){
        myservo = Servo(PA_11);
        wheel = QEI(PA_12, PB_0, NC, 24, QEI::X4_ENCODING);
    };
    iE = 0;
    dE = 0;
    Kp = 0.01378;
    Ki = 0.0006875;
    Kd = 0.00011;
}

void Kinesthetix::control(float desired) {
        // Time since timer was last reset
        t.start();                      // Start timer
        dt = t.read() * 0.7 + dt * 0.3; // Numerical smoothing as it's quite noisy
        t.reset();                      // Reset timer to 0, but keep counting
        pos = wheel.getPulses();
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
        myservo = 0.5 + Kntrl;
}     