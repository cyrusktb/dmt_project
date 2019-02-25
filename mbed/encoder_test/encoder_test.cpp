#include "mbed.h"

Serial pc(USBTX, USBRX);

volatile char i1a, i1b, i2a, i2b, i3a, i3b;


InterruptIn en1a(PA_12);
InterruptIn en1b(PB_0);

InterruptIn en2a(PA_8);
InterruptIn en2b(PA_7);

InterruptIn en3a(PA_3);
InterruptIn en3b(PA_1);

void tog1a() {
    i1a++;
}

void tog1b() {
    i1b++;
}

void tog2a() {
    i2a++;
}

void tog2b() {
    i2b++;
}

void tog3a() {
    i3a++;
}

void tog3b() {
    i3b++;
}

int main() {
    pc.baud(115200);
    i1a = 0;
    i1b = 0;
    i2a = 0;
    i2b = 0;
    i3a = 0;
    i3b = 0;

    pc.printf("----- Starting -----\n");
    
    en1a.rise(&tog1a);
    en1a.fall(&tog1a);

    en1b.rise(&tog1b);
    en1b.fall(&tog1b);

    en2a.rise(&tog2a);
    en2a.fall(&tog2a);

    en2b.rise(&tog2b);
    en2b.fall(&tog2b);

    en3a.rise(&tog3a);
    en3a.fall(&tog3a);

    en3b.rise(&tog3b);
    en3b.fall(&tog3b);

    pc.printf("----- Ready -----\n");

    while(true) {
        pc.printf("1: [%d, %d], 2: [%d, %d], 3: [%d, %d]\n", 
                  i1a, i1b, i2a, i2b, i3a, i3b);
        wait(0.1);
    }
}
