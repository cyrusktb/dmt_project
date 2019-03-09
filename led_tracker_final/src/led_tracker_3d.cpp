#include "led_tracker_3d.hpp"

// Useful vector functions
float dot(cv::Vec3f a, cv::Vec3f b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float mag(cv::Vec3f a) {
    return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}
LedTracker3D::LedTracker3D() 
        :thread_(&LedTracker3D::loop, this), 
         running_(true),
         trackers_{ LedTracker2D(LEFT_CAMERA_PARAMS), 
                    LedTracker2D(RIGHT_CAMERA_PARAMS) } {
    // Set position and rotation of the cameras
    trackers_[0].pos = LEFT_CAMERA_POS;
    trackers_[0].rot = LEFT_CAMERA_ROT;

    trackers_[1].pos = RIGHT_CAMERA_POS;
    trackers_[1].pos = RIGHT_CAMERA_POS;
}

LedTracker3D::~LedTracker3D() {
    std::cout << "Shutting down \033[31mLedTracker3D\033[0m...";

    // Tell thread to stop running and wait for it to finish cleanly
    running_ = false;
    thread_.join();

    std::cout << "\t\t[\033[1;32mOK\0330m]" << std::endl;
}

void LedTracker3D::get_imgs(cv::Mat &left, cv::Mat &right) {
    // Prevent images being changed mid-copy
    std::lock_guard<std::mutex> lock(img_mutex_);
    
    left = imgs_[0];
    right = imgs_[1];
}

void LedTracker3D::get_glove_pos(cv::Point3f &pos, cv::Point3f &ypr) {
    pos = glove_pos_;
    ypr = glove_ypr_;
};

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

        // Pass the identified points to the 2d trackers
        // so that they may update position weighting
        trackers_[0].set_prev_leds(pot_leds_[0]);
        trackers_[1].set_prev_leds(pot_leds_[1]);
    }
    
    // Clean up memory
    delete left_thread;
    delete right_thread;
}

void LedTracker3D::find_glove_pos() {
    // Vectors of 3d points split into two colours
    // Access with points[g=0/b=1][point]
    std::vector<cv::Point3f> points[2];

    // Reference to which pot_led we're looking at
    // Access with point_its[g=0/b=1][point][l=0/r=1]
    std::vector<std::array<int, 2>> point_its[2];
    
    // Find the list of 3d points
    find_pot_3d_points(&points[0], &point_its[0], 
                       &points[1], &point_its[1]);
    
    // Consider the posibility of no points of the 
    // correct colour on the screen
    if(!points[0].size() && !points[1].size())
        return;

    // Find the combination with the least error
    float err = 99999;
    int err_its[3] = {0, 0, 0};

    // Loop through each combination of LEDs
    for(int i = 0; i < points[0].size(); i++) {
        for(int j = 0; j < points[1].size(); i++) {
            // If the point has a large enough threshold to be on the left
            if(pot_leds_[1][point_its[1][j][0]].total_weight[1] >=
               MIN_TOTAL_WEIGHT                              || 
               pot_leds_[1][point_its[1][j][1]].total_weight[1] >= 
               MIN_TOTAL_WEIGHT) {
                for(int k = 0; k < points[1].size(); k++) {
                    // Ignore duplicate points
                    if(k == j) 
                        continue;

                    // Ignore points not possibly on the right
                    if(pot_leds_[1][point_its[1][k][0]].total_weight[2] <
                       MIN_TOTAL_WEIGHT                              || 
                       pot_leds_[1][point_its[1][k][1]].total_weight[2] < 
                       MIN_TOTAL_WEIGHT)
                        continue;
                    
                    // Calculate error in pose given these three points
                    float e = calculate_error(points[0][i],
                                              points[1][j],
                                              points[1][k]);
                    
                    // Update best solution
                    if(e < err) {
                        err = e;
                        err_its[0] = i; 
                        err_its[1] = j; 
                        err_its[2] = k;
                    }
                }
            }
            // We don't need to check if the point is on the right as it
            // will be compared with other potential lefts above
        }
    }

    // Calculate the pose of the triangle made by the three points
    // We want to find a matrix T such that T*G0 = G, T*B_L0 = B_L 
    // and T*B_R0 = B_R

    cv::Point3f best_points[3];
    best_points[0] = points[0][err_its[0]];
    best_points[1] = points[1][err_its[1]];
    best_points[2] = points[1][err_its[2]];

    // Calculate the vector G->B)L
    cv::Vec3f GBL(BLUE_L_POS - GREEN_POS);
    // Calculate the new vector G'->BL'
    cv::Vec3f new_GBL(best_points[1] - best_points[0]);

    // Calculate the rotation matrix between the two vectors
    // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
    
    cv::Vec3f v = GBL.cross(new_GBL);
    float c = dot(GBL, new_GBL);

    cv::Matx33f I{1, 0, 0,
                  0, 1, 0,
                  0, 0, 1};
    
    cv::Matx33f Vx{0,   -v[2], v[1],
                   v[2], 0,   -v[0],
                  -v[1], v[0], 0};
    
    cv::Matx33f R = I + Vx + (1 / (1 + c)) * Vx * Vx;

    // With the rotation matrix found, we just need to find the translation
    // To account for some errors, average the three
    glove_pos_ = (best_points[0] - R * GREEN_POS +
                   best_points[1] - R * BLUE_L_POS +
                   best_points[2] - R * BLUE_R_POS) / 3;

    // Convert rotation matrix to yaw, pitch, roll
    glove_ypr_.x = atan2(R(1, 0), R(0,0));
    glove_ypr_.y = atan2(-R(2, 0), 
                           sqrt(pow(R(2, 1), 2) + pow(R(2,2), 2)));
    glove_ypr_.z = atan2(R(2,1), R(2, 2));

    // Delete potential leds that were not chosen
    for(char i = 0; i < 2; i++) {
        for(int p = 0; p < pot_leds_[i].size(); p++) {
            if(p != point_its[0][err_its[0]][i] &&
               p != point_its[1][err_its[1]][i] &&
               p != point_its[1][err_its[2]][i]) {
                pot_leds_[i].erase(pot_leds_[i].begin() + p);
                p--;
            }
        }
    }


}

void LedTracker3D::find_pot_3d_points(std::vector<cv::Point3f> *green,
                                      std::vector<std::array<int, 2>> *green_its,
                                      std::vector<cv::Point3f> *blue,
                                      std::vector<std::array<int, 2>> *blue_its) {
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
                float dist;
                cv::Point3f c;
                intersect_rays(pot_leds_[0][i].ray, 
                               pot_leds_[1][j].ray,
                               &c,
                               &dist);
                
                // Store the result
                green->push_back(c);
                green_its->push_back({{i, j}});
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
                float dist;
                cv::Point3f c;
                intersect_rays(pot_leds_[0][i].ray, 
                               pot_leds_[1][j].ray,
                               &c,
                               &dist);
                
                // Store the result
                blue->push_back(c);
                blue_its->push_back({{i, j}});
            }
        }
    }
}

float LedTracker3D::calculate_error(cv::Point3f green,
                                    cv::Point3f blue_l,
                                    cv::Point3f blue_r) {
    // Calculate the length of each side of the triangle formed by 3 points
    float sides[3];
    
    sides[0] = mag(green-blue_l);
    sides[1] = mag(green-blue_r);
    sides[2] = mag(blue_l - blue_r);

    // Calculate the average error in lengths
    float err = (sides[0] - GBL_LEN 
              + sides[1] - GBR_LEN 
              + sides[2] - BLBR_LEN) / 3.f;

    return err;
}

void LedTracker3D::intersect_rays(cv::Vec3f l, 
                                  cv::Vec3f r, 
                                  cv::Point3f *center,
                                  float *distance) {
    // left starts at camera_pos_[0], right starts at camera_pos_[1]
    cv::Vec3f c = l - r;
    
    // Calculate points on each line where the lines are closest
    cv::Vec3f l_p = trackers_[0].pos;
    l_p += ((dot(l, r)*dot(r, c) + dot(l, c)*dot(r, r))
           / (dot(l, l)*dot(r, r) - dot(l, r)*dot(l, r))) * l;
                    
    cv::Vec3f r_p = trackers_[1].pos;
    r_p += ((dot(l, r)*dot(l, c) + dot(r, c)*dot(l, l))
           / (dot(l, l)*dot(r, r) - dot(l, r)*dot(l, r))) * r;

    // Find vector between points
    cv::Vec3f v = l_p - r_p;
    
    // Find distance and center point
    *distance = mag(v);
    *center = r_p + 0.5*v;
}
