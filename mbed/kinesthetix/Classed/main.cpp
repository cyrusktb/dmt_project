#include "mbed.h"
#include "Servo.h"
#include "QEI.h"
#include "Kinesthetix.hpp"

Serial pc(USBTX, USBRX);
Kinesthetix kin(MIDDLE);

int main(){
    pc.baud(9600); 
    kin.control(300);
    
    pc.printf("\rPulses:  desShift:  propCntrl: \r\n");        
    }