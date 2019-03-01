#include "mbed.h"
#include "Servo.h"
#include "QEI.h"

volatile int pos;

Serial pc(USBTX, USBRX);
Servo myservo(PA_4);
Timer t;

QEI wheel(PA_1, PA_3, NC, 24, QEI::X4_ENCODING);

int main() {
    t.start();
    pc.baud(9600); //Change this as desired for thine Personal Computer
    //This variable must change depending on what the claw position is 
    int desired = 200; 
    volatile float desShift; 
    float Kntrl, prevT, curT, dt, iE = 0;
    float Kp = 0.002, Ki = 0.0003;
    while(true) {
        prevT = curT;
        curT = t.read();
        dt = curT - prevT;
        
        pos = wheel.getPulses();
        desShift = desired - pos; //The "Normalised" Position to stop the motor
        iE += desShift * dt;
        Kntrl = 0.5 + Kp * desShift;// + Ki * iE ; //Proportional Control
        myservo = Kntrl;
        pc.printf("\rPulses: %i desShift: %f propCntrl: %f \r\n", pos, desShift, Kntrl);        
        //This code ultimately receives a desired position "desired" from the 
        //claw fingers which then make the servos operate towards that position
        //PROPORTIONALLY CONTROLLED BOI;
    }
}

// Servo pins   A_11 B_4 A_4
// Encoder pins A_12 B_0 A_8 A_7 A_3 A_1