#include "led_tracker_2d.hpp"

LedTracker3D::LedTracker3D() :thread_(&LedTracker3D::loop, this) {
    // ctor
}

LedTracker3D::~LedTracker3D() {
    std::cout << "Shutting down \033[31mLedTracker3D\033[0m...";

    // Tell thread to stop running and wait for it to finish cleanly
    running_ = false;
    loop_thread_.join();

    std::cout << "\t\t[\033[1;32mOK\0330m]" << std::endl;
}

void LedTracker3D::get_imgs(cv::Mat &left, cv::Mat &right) {
    // Prevent images being changed mid-copy
    std::lock_guard<std::mutex> lock(img_mutex_);
    
    left = imgs_[0];
    right = imgs_[1];
}

void LedTracker3D::get_glove_pos(cv::Point3f &pos, cv::Point3f &rot) {
    
}

void LedTracker3D::loop() {
    // Create lambda to pass to threads easily
    auto data_func = [this](short tracker_num){
        // Get the next frame's set of points for processing
        auto new_pot_leds = this->trackers_[tracker_num].get_points();
        
        // Wait for the current set to be obsolete before overwriting
        std::lock_guard<std::mutex> lock(this->pot_mutex_);

        // Overwrite
        this->pot_leds_[tracker_num].clear();
        this->pot_leds_[tracker_num] = new_pot_leds;
    };

    // Create a thread for each 2d tracker, running the above lambdas
    std::thread *left_thread = new std::thread(data_func, 0);
    std::thread *right_thread = new std::thread(data_func, 1);

    // Wait for both threads to finish
    left_thread->join();
    right_thread->join();

    // Loop until told to stop to allow us to exit cleanly
    while(running_) {
        // Create lock to prevent threads from finishing before we finish
        // processing the data
        std::unique_lock<std::mutex> lock(pot_mutex_);

        // Start the threads processing the next set of images,
        // before processing the current set of data
        left_thread = new std::thread(data_func, 0);
        right_thread = new std::thread(data_func, 1);

        // Find the position of the glove from the previous data set
        find_glove_pos();

        // Release the lock
        lock.unlock();

        // Wait for the threads to finish before the next iteration
        left_thread->join();
        right_thread->join();

        // Obtain lock on potential LEDS
    }
    
    // Clean up memory
    delete left_thread;
    delete right_thread;
}

void LedTracker3D::find_glove_pos() {
    // Vectors of 3d points split into two colours
    std::vector<cv::Point3f> points[2];

    // Reference to which pot_led we're looking at
    std::vector<int*> point_its[3];
    
    // Find the list of 3d points
    find_pot_3d_points(&points[0], &point_its[0], 
                       &points[1], &point_its[1]);
    
    // Consider the posibility of no points of the 
    // correct colour on the screen
    if(!points[0].size() && !points[1].size())
        return;

    // Find the combination with the least error
    float err = 99999;
    int[3] err_its = {0, 0, 0};

    // Loop through each combination of LEDs
    for(int i = 0; i < points[0].size(); i++) {
        for(int j = 0; j < points[1].size(); i++) {
            // If the point has a large enough threshold to be on the left
            if(pot_leds_[0][point_its[j][0]].total_weight[1] >=
               MIN_TOTAL_WEIGHT                              || 
               pot_leds_[1][point_its[j][1]].total_weight[1] >= 
               MIN_TOTAL_WEIGHT) {
                for(int k = 0; k < points[1].size() {
                    // Ignore duplicate points
                    if(k == j) 
                        continue;

                    // Ignore points not possibly on the right
                    if(pot_leds_[0][point_its[k][0]].total_weight[2] <
                       MIN_TOTAL_WEIGHT                              || 
                       pot_leds_[1][point_its[k][1]].total_weight[2] < 
                       MIN_TOTAL_WEIGHT)
                        continue;
                    
                    // Calculate error in pose given these three points
                    float e = calculate_pose_error(points[0][i],
                                                   points[1][j],
                                                   points[1][k]);
                    
                    // Update best solution
                    if(e < err) {
                        err = e;
                        err_its = {i, j, k};
                    }
                }
            }
            // We don't need to check if the point is on the right as it
            // will be compared with other potential lefts above
        }
    }

    // Calculate the pose of the triangle made by the three points
}

void LedTracker3D::find_pot_3d_points(std::vector<cv::Point3f> *green,
                                      std::vector<int*> *green_its,
                                      std::vector<cv::Point3f> *blue
                                      std::vector<int*> blue_its) {
    // Loop through each combination of contours to find best match
    for(int i = 0; i < pot_leds_[0].size(); i++) {
        for(int j = 0; j < pot_leds_[1].size(); j++) {
            // Check that both contours are the same colour

            // left is green
            if(pot_leds_[0][i].total_weight[0] >
               pot_leds_[0][i].total_weight[1] &&
               pot_leds_[0][i].total_weight[0] >
               pot_leds_[0][i].total_weight[2]) {
                // right is blue
                if(pot_leds_[1][j].total_weight[0] <
                   pot_leds_[1][j].total_weight[1] ||
                   pot_leds_[1][j].total_weight[0] <
                   pot_leds_[1][j].total_weight[2])
                    continue;
                // Both contours are green
                
                // Find intersection point
                cv::Point3f c = intersect_rays(pot_leds_[0].ray, 
                                               pot_leds_[1].ray);
                
                // Store the result
                green->push_back(c);
                green_its.push_back({i, j});
            }
            // left is blue
            else {
                // right is green
                if(pot_leds_[1][j].total_weight[0] >
                   pot_leds_[1][j].total_weight[1] &&
                   pot_leds_[1][j].total_weight[0] >
                   pot_leds_[1][j].total_weight[2])
                    continue;
                // Both contours are blue
                
                // Find intersection point
                cv::Point3f c = intersect_rays(pot_leds_[0].ray, 
                                               pot_leds_[1].ray);
                
                // Store the result
                blue->push_back(c);
                blue_its.push_back({i, j});
            }
        }
    }
}
