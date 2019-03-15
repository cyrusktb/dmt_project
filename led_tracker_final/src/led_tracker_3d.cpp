#include "led_tracker_3d.hpp"

LedTracker3D::LedTracker3D() 
        :thread_(&LedTracker3D::loop, this), 
         running_(true),
         trackers_{ LedTracker2D(LEFT_CAMERA_PARAMS), 
                    LedTracker2D(RIGHT_CAMERA_PARAMS) } {
    // Set position and rotation of the cameras
    trackers_[0].pos = LEFT_CAMERA_POS;
    trackers_[0].rot = LEFT_CAMERA_ROT;

    trackers_[1].pos = RIGHT_CAMERA_POS;
    trackers_[1].rot = RIGHT_CAMERA_ROT;
}

LedTracker3D::~LedTracker3D() {
    std::cout << "Shutting down \033[31mLedTracker3D\033[0m...";
    // Tell thread to stop running and wait for it to finish cleanly
    running_ = false;
    thread_.join();

    std::cout << "\t\t[\033[1;32mOK\033[0m]" << std::endl;
}

void LedTracker3D::get_imgs(cv::Mat &left, cv::Mat &right) {
    // Prevent images being changed mid-copy
    std::lock_guard<std::mutex> lock(img_mutex_);
    
    imgs_[0].copyTo(left);
    imgs_[1].copyTo(right);
}

void LedTracker3D::get_glove_pos(cv::Point3f &pos, cv::Point3f &ypr) {
    pos = glove_pos_;
    ypr = glove_ypr_;
};

void LedTracker3D::loop() {
    // Create lambda to pass to threads easily
    auto data_func = [this](short tracker_num) {
        // Get the next frame's set of points for processing
        auto new_pot_leds = this->trackers_[tracker_num].get_points();

        // Wait for the current set to be obsolete before overwriting
        std::lock_guard<std::mutex> lock(this->pot_mutex_);

        // Overwrite
        this->pot_leds_[tracker_num].clear();
        this->pot_leds_[tracker_num] = new_pot_leds;

        // Prevent images being copied mid change
        std::lock_guard<std::mutex> lock_img(img_mutex_);

        // Update image
        trackers_[tracker_num].get_img(imgs_[tracker_num]);
    };

    // Create a thread for each 2d tracker, running the above lambdas
    std::thread *left_thread = new std::thread(data_func, 0);
    std::thread *right_thread = new std::thread(data_func, 1);

    // Wait for both threads to finish
    left_thread->join();
    right_thread->join();

    time_t time_;
    time(&time_);
    srand(time_);


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

        // Draw circles on chosen leds (for debugging)
        for(char i = 0; i < 2; i++) {
            for(char j = 0; j < pot_leds_[i].size(); j++) {
                cv::circle(imgs_[i],
                           pot_leds_[i][j].center,
                           5,
                           cv::Scalar(100, 100, 255),
                           2);
            }
        }

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
    if(!points[0].size() || !points[1].size())
        return;

    // Find the combination with the least error
    float err = 99999;
    int err_its[3] = {0, 0, 0};

    // Loop through each combination of LEDs
    for(int i = 0; i < points[0].size(); i++) {
        for(int j = 0; j < points[1].size(); j++) {
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

    Triangle glove_tri;
    glove_tri.A = best_points[0];
    glove_tri.B = best_points[1];
    glove_tri.C = best_points[2];

    Triangle origin_tri;
    origin_tri.A = GREEN_POS;
    origin_tri.B = BLUE_L_POS;
    origin_tri.C = BLUE_R_POS;

    // Calculate the rotation matrix
    cv::Matx33f R;
    get_rotation_matrix(R, origin_tri, glove_tri);

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

    std::vector<PotentialLed> kept_pot_leds_[2];

    // Delete potential leds that were not chosen
    for(char i = 0; i < 2; i++) {
        for(int p = 0; p < pot_leds_[i].size(); p++) {
            if(p == point_its[0][err_its[0]][i] ||
               p == point_its[1][err_its[1]][i] ||
               p == point_its[1][err_its[2]][i]) {
                kept_pot_leds_[i].push_back(pot_leds_[i][p]);
            }
        }
    }
    pot_leds_[0] = kept_pot_leds_[0];
    pot_leds_[1] = kept_pot_leds_[1];
}

void LedTracker3D::find_pot_3d_points(std::vector<cv::Point3f> *green,
                                      std::vector<std::array<int, 2>> *green_its,
                                      std::vector<cv::Point3f> *blue,
                                      std::vector<std::array<int, 2>> *blue_its) {
    // Loop through each combination of contours to find best match
    for(int i = 0; i < pot_leds_[0].size(); i++) {
        for(int j = 0; j < pot_leds_[1].size(); j++) {
            // Find intersection point
            float dist;
            cv::Point3f *c = new cv::Point3f;
            intersect_rays(pot_leds_[0][i].ray, 
                           pot_leds_[1][j].ray,
                           c,
                           &dist);

            // If the result is invalid, or dist is too large, ignore
            if(!c || dist > INTERSECTION_TOLERANCE || c->z < 0)
                continue;
            
            cv::Scalar col(rand() % 255, rand() % 255, rand() % 255);

            // Store the result
            green->push_back(*c);
            green_its->push_back({{i, j}});
            
            blue->push_back(*c);
            blue_its->push_back({{i, j}});
/*
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
                cv::Point3f *c = new cv::Point3f;
                intersect_rays(pot_leds_[0][i].ray, 
                               pot_leds_[1][j].ray,
                               c,
                               &dist);

                // If the result is invalid, or dist is too large, ignore
                if(!c || dist > INTERSECTION_TOLERANCE || c->z < 0)
                    continue;
                
                cv::Scalar col(rand() % 255, rand() % 255, rand() % 255);

                cv::circle(imgs_[0],
                           pot_leds_[0][i].center,
                           5,
                           col,
                           2);
                cv::circle(imgs_[1],
                           pot_leds_[1][j].center,
                           5,
                           col,
                           2);

                // Store the result
                green->push_back(*c);
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
                cv::Point3f *c = new cv::Point3f;
                intersect_rays(pot_leds_[0][i].ray, 
                               pot_leds_[1][j].ray,
                               c,
                               &dist);

                // If the result is invalid, or dist is too large, ignore
                if(!c || dist > INTERSECTION_TOLERANCE || c->z < 0)
                    continue;
                
                cv::Scalar col(rand() % 255, rand() % 255, rand() % 255);

                cv::circle(imgs_[0],
                           pot_leds_[0][i].center,
                           5,
                           col,
                           2);
                cv::circle(imgs_[1],
                           pot_leds_[1][j].center,
                           5,
                           col,
                           2);

                // Store the result
                blue->push_back(*c);
                blue_its->push_back({{i, j}});
            } */
        }
    }
}

float LedTracker3D::calculate_error(cv::Point3f green,
                                    cv::Point3f blue_l,
                                    cv::Point3f blue_r) {
    // Calculate the length of each side of the triangle formed by 3 points
    float sides[3];
    
    // The triangle found appears to be slightly larger than measured,
    // So calculate error to be the variation in ratio
    sides[0] = mag(green-blue_l) / GBL_LEN;
    sides[1] = mag(green-blue_r) / GBR_LEN;
    sides[2] = mag(blue_l-blue_r) / BLBR_LEN;

    float avg = (sides[0] + sides[1] + sides[2]) / 3.f;

    // Find the standard deviation
    float sd = sqrt((pow(sides[0] - avg, 2) 
                   + pow(sides[1] - avg, 2) 
                   + pow(sides[2] - avg, 2))
                   / 3.f);

    // Calculate the coefficient of variation
    float cv = sd / avg;

    return cv;
}

void LedTracker3D::intersect_rays(cv::Vec3f l, 
                                  cv::Vec3f r, 
                                  cv::Point3f *center,
                                  float *distance) {
    // trackers_[0].pos + mu_l * l = trackers_[1].pos + mu_l * r + lam * (lxr)
    // Rearranging to matrix equation gives:
    // M * (mu_r, lam, mu_l)' = (trackers_[0].pos - trackers_[1].pos)
    // With the ith row of M = [r_i, (r x l)_i, -l_i]
    float mu_l, mu_r, lam;

    cv::Vec3f cr = cross(l, r);

    // Normalise vectors
    l /= mag(l);
    r /= mag(r);
    cr /= mag(cr);

    // Formulate matrix equation
    cv::Matx33f M = { r[0], cr[0], -l[0],
                      r[1], cr[1], -l[1],
                      r[2], cr[2], -l[2] };
    
    cv::Vec3f scalars = M.inv() * (trackers_[0].pos - trackers_[1].pos);
    mu_r = scalars[0];
    lam = scalars[1];
    mu_l = scalars[2];
    
    // Calculate center
    
    *center = (cv::Vec3f)trackers_[1].pos + mu_r * r + cr * lam / 2;
    *distance = lam;
}
