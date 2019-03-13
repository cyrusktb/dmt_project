#include "led_tracker_3d.hpp"

#define _2DTEST

int main(int argc, char **argv) {
    cv::Mat left, right;
#ifdef _3DTEST
    LedTracker3D tracker;
    
    // Wait a moment for tracker to setup
    while(left.size() == cv::Size(0,0) ||
          right.size() == cv::Size(0,0)) {
        tracker.get_imgs(left, right);
    }
#endif
#ifdef _2DTEST
    LedTracker2D left_t(LEFT_CAMERA_PARAMS);
    LedTracker2D right_t(RIGHT_CAMERA_PARAMS);


    auto func = [](LedTracker2D *tracker, cv::Mat *mat) {
        auto new_pots = tracker->get_points();
        tracker->get_img(*mat);

        std::cout << "\033[31m" << new_pots.size() 
                  << "\033[0m" << std::endl;

        for(char i = 0; i < new_pots.size(); i++) {
            cv::circle(*mat, 
                       new_pots[i].center, 
                       5, 
                       cv::Scalar(128, 128, 255), 
                       3);
        }
        
        // Find the best weights
        float best_weight[3] = {0, 0, 0};
        float best_loc[3] = {0, 0, 0};

        for(char i = 0; i < new_pots.size(); i++) {
            for(char j = 0; j < 3; j++) {
                if(new_pots[i].total_weight[j] > best_weight[j]) {
                    best_weight[j] = new_pots[i].total_weight[j];
                    best_loc[j] = i;
                }
            }
        }
        std::vector<PotentialLed> best;
        if(new_pots.size()) {
            for(char i = 0; i < 3; i++) {
                best.push_back(new_pots[best_loc[i]]);
            }
        }
        tracker->set_prev_leds(best);
    };
#endif
    while(true) {
#ifdef _2DTEST
        std::thread a(func, &left_t, &left);
        std::thread b(func, &right_t, &right);

        a.join();
        b.join();
#endif
#ifdef _3DTEST
        tracker.get_imgs(left, right);
        
        cv::Point3f pos, ypr;
        tracker.get_glove_pos(pos, ypr);

        std::cout << "\033[31m"
                  << "X: " << pos.x * 100
                  << " Y : " << pos.y * 100
                  << " Z : " << pos.z * 100
                  << " YAW: " << ypr.x * 180
                  << "\033[0m" << std::endl;
#endif
       
        // Display the image
        cv::imshow("LedTracker - left", left);
        cv::imshow("LedTracker - right", right);

        // Give 30ms to update the image display
        // 27 is the ESC key
        if(cv::waitKey(30) == 27) break;
    }
}
