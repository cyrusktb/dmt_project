#include "mbed.h"
#include "FSR.h"

Serial pc(USBTX, USBRX);
FSR fsr(PA_2, 3.3); // a 10k resistor is used

int main(){
    pc.baud(9600);
    while (1) {
    pc.printf("The raw data is %f\r\n", fsr.readRaw());
    pc.printf("The resistance of the FSR is %f\r\n", fsr.readFSRResistance());
    pc.printf("The weight on the FSR is %f\r\n\n", fsr.readWeight());
    wait(0.3); //just here to slow down the output for easier reading
    }
}