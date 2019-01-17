#include "mbed.h"

#define MAX 0.25f
#define MIN 0.15f
#define STEP 0.01f
#define CYCLE_DURATION_S 3.0f

PwmOut left_erm(PA_6);
PwmOut right_erm(PA_7);

float left_duty;
float right_duty;

int main() {
    left_erm.period(1.0f / 10000.0f);
    right_erm.period(1.0f / 10000.0f);
    
    left_duty = MIN;
    right_duty = MAX;
    
    bool dir = 0;

    while(1) {
        if(dir) {
            left_duty -= STEP;
            right_duty += STEP;

            wait_us(STEP  * 1000000.f * CYCLE_DURATION_S / (MAX-MIN));

            if(left_duty <= MIN) {
                dir = 0;
            }
        }
        else {
            left_duty += STEP;
            right_duty -= STEP;

            wait_us(STEP  * 1000000.f * CYCLE_DURATION_S / (MAX-MIN));

            if(left_duty >= MAX) {
                dir = 1;
            }
        }
        left_erm.write(left_duty);
        right_erm.write(right_duty);
    }
}
