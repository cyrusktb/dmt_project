#include "led_tracker_3d.hpp"

int main(int argc, char **argv) {
    cv::Mat left, right;

    LedTracker3D tracker;
    
    // Wait a moment for tracker to setup
    while(left.size() == cv::Size(0,0) ||
          right.size() == cv::Size(0,0)) {
        tracker.get_imgs(left, right);
    }

    while(true) {
        tracker.get_imgs(left, right);
        
        // Display the image
        cv::imshow("LedTracker - left", left);
        cv::imshow("LedTracker - right", right);
/*
        std::cout << "\033[31m"
                  << "G: " << tracker.get_led_pos(LedNum::GREEN_LED)
                  << "  B_L: " << tracker.get_led_pos(LedNum::BLUE_LED_L)
                  << "  B_R: " << tracker.get_led_pos(LedNum::BLUE_LED_R)
                  << "\033[0m" << std::endl;
 */       
        // Give 30ms to update the image display
        // 27 is the ESC key
        if(cv::waitKey(30) == 27) break;
    }
}
