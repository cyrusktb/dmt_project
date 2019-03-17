#ifndef __KINESTHETIC_CONTROLLER_H__
#define __KINESTHETIC_CONTROLLER_H__

#include "kinesthetix.h"

class KinestheticController {
public:
    KinestheticController();
    ~KinestheticController();

    // Sets the target to be passed into the control function of the finger
    void set_finger_target(char finger, float target) {
        if(finger > 2) return;
        targets_[finger] = target;
    };

    // Get the position of a finger servo motor
    float get_finger_pos(char finger) {
        if(finger > 2) return -1;
        // TODO: get the finger position and return it
        targets_[finger]/*.get_pos()*/;
        return 5;
    }

    // Update PID control of each finger
    void update();
private:
    // Servos representing the three fingers
    // 0 is thumb, 1 is index, 2 is middle
    Kinesthetix fingers_[3];

    // Target positions of the fingers
    float targets_[3];
};

#endif // __KINESTHETIC_CONTROLLER_H__
