#include "kinesthetic_controller.h"

KinestheticController::KinestheticController()
        :fingers_{ Kinesthetix(FingerType::THUMB), 
                   Kinesthetix(FingerType::INDEX), 
                   Kinesthetix(FingerType::MIDDLE) },
         targets_{ 0,
                   0,
                   0 } {
    // ctor
}

KinestheticController::~KinestheticController() {
    // dtor
}

void KinestheticController::update() {
    // Loop through each finger and update it
    for(char i = 0; i < 3; i++) {
        fingers_[i].control(targets_[i]);
    }
}
