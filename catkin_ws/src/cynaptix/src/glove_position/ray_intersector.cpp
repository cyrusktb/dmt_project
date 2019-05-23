#include "glove_position/ray_intersector.hpp"

RayIntersector::RayIntersector() :nh_("~"), tf_listener_(tf_buffer_) {
    if(!nh_.param<float>("max_message_delay", message_delay_, 1/60.f))
        ROS_WARN("Failed to get param 'max_message_delay', defaulting to %f",
                 message_delay_);

    if(!nh_.param<float>("intersection_tolerance", intersection_tol_, 0.05))
        ROS_WARN("Failed to get param 'intersection_tolerance', defaulting to %f",
        intersection_tol_);

    if(!nh_.param<float>("glove_boundary_min_x", min_x_, -0.25))
        ROS_WARN("Failed to get param 'glove_boundary_min_x', defaulting to %f", 
                 min_x_);
    if(!nh_.param<float>("glove_boundary_max_x", max_x_, 0.25))
        ROS_WARN("Failed to get param 'glove_boundary_max_x', defaulting to %f", 
                 max_x_);

    if(!nh_.param<float>("glove_boundary_min_y", min_y_, -0.25))
        ROS_WARN("Failed to get param 'glove_boundary_min_y', defaulting to %f", 
                 min_y_);
    if(!nh_.param<float>("glove_boundary_max_y", max_y_, 0.25))
        ROS_WARN("Failed to get param 'glove_boundary_max_y', defaulting to %f", 
                 max_y_);

    if(!nh_.param<float>("glove_boundary_min_z", min_z_, -0.25))
        ROS_WARN("Failed to get param 'glove_boundary_min_z', defaulting to %f", 
                 min_z_);
    if(!nh_.param<float>("glove_boundary_max_z", max_z_, 0.25))
        ROS_WARN("Failed to get param 'glove_boundary_max_z', defaulting to %f", 
                 max_z_);

    // Get the names of the topicsto subscribe to
    std::string left_topic, right_topic;
    if(!nh_.getParam("left_ray_topic", left_topic))
        ROS_FATAL("Failed to get param 'left_ray_topic'");
    if(!nh_.getParam("right_ray_topic", right_topic))
        ROS_FATAL("Failed to get param 'right_ray_topic'");

    left_sub_ = nh_.subscribe(left_topic, 1, 
                              &RayIntersector::left_callback, 
                              this);

    right_sub_ = nh_.subscribe(right_topic, 1, 
                               &RayIntersector::right_callback, 
                               this);

    // Debug mode
    if(nh_.param<bool>("debug", debug_, false)) {
        ROS_INFO("Running in debug mode!");
    }
}

RayIntersector::~RayIntersector() {
    // dtor
}

void RayIntersector::left_callback(const cynaptix::LedRayArray& msg){
    // Store msg
    left_rays_ = msg;

    // Remove duplicate rays
    for(int i = 0; i < left_rays_.greens.size(); i++) {
        for(int j = i+1; j < left_rays_.greens.size(); j++) {
            cv::Vec3f r1, r2;
            r1[0] = left_rays_.greens[i].x;
            r1[1] = left_rays_.greens[i].y;
            r1[2] = left_rays_.greens[i].z;
            r2[0] = left_rays_.greens[j].x;
            r2[1] = left_rays_.greens[j].y;
            r2[2] = left_rays_.greens[j].z;

            if(mag(r1-r2) < 0.02) {
                left_rays_.greens.erase(left_rays_.greens.begin() +j);
                j--;
            }
        }
    }
    for(int i = 0; i < left_rays_.blues.size(); i++) {
        for(int j = i+1; j < left_rays_.blues.size(); j++) {
            cv::Vec3f r1, r2;
            r1[0] = left_rays_.blues[i].x;
            r1[1] = left_rays_.blues[i].y;
            r1[2] = left_rays_.blues[i].z;
            r2[0] = left_rays_.blues[j].x;
            r2[1] = left_rays_.blues[j].y;
            r2[2] = left_rays_.blues[j].z;

            if(mag(r1-r2) < 0.02) {
                left_rays_.blues.erase(left_rays_.blues.begin() +j);
                j--;
            }
        }
    }
        
    intersect_rays();
}

void RayIntersector::right_callback(const cynaptix::LedRayArray& msg){
    // Store msg
    right_rays_ = msg;

    // Remove duplicate rays
    for(int i = 0; i < right_rays_.greens.size(); i++) {
        for(int j = i+1; j < right_rays_.greens.size(); j++) {
            cv::Vec3f r1, r2;
            r1[0] = right_rays_.greens[i].x;
            r1[1] = right_rays_.greens[i].y;
            r1[2] = right_rays_.greens[i].z;
            r2[0] = right_rays_.greens[j].x;
            r2[1] = right_rays_.greens[j].y;
            r2[2] = right_rays_.greens[j].z;

            if(mag(r1-r2) < 0.02) {
                right_rays_.greens.erase(right_rays_.greens.begin() +j);
                j--;
            }
        }
    }
    for(int i = 0; i < right_rays_.blues.size(); i++) {
        for(int j = i+1; j < right_rays_.blues.size(); j++) {
            cv::Vec3f r1, r2;
            r1[0] = right_rays_.blues[i].x;
            r1[1] = right_rays_.blues[i].y;
            r1[2] = right_rays_.blues[i].z;
            r2[0] = right_rays_.blues[j].x;
            r2[1] = right_rays_.blues[j].y;
            r2[2] = right_rays_.blues[j].z;

            if(mag(r1-r2) < 0.02) {
                right_rays_.blues.erase(right_rays_.blues.begin() +j);
                j--;
            }
        }
    }

    intersect_rays();
}

void RayIntersector::intersect_rays() {
    // Attempt to get camera transforms
    try {
        left_cam_tf_ = tf_buffer_.lookupTransform("world",
                                                  "left_camera_frame", 
                                                  ros::Time(0));
        right_cam_tf_ = tf_buffer_.lookupTransform("world",
                                                   "right_camera_frame", 
                                                   ros::Time(0));
    }
    catch(tf2::TransformException &ex) {
        ROS_ERROR("Failed to get camera transforms:\n%s", ex.what());
        return;
    }

    // Store the Point and the Distance between the rays
    std::vector<LedPoint> g_p;

    // Loop through all green rays and find all potential points
    for(int l = 0; l < left_rays_.greens.size(); l++) {
        for(int r = 0; r < right_rays_.greens.size(); r++) {
            g_p.push_back(LedPoint());
            g_p.back().ray_id_1 = l;
            g_p.back().ray_id_2 = r;
            // Check intersection, delete if no intersection
            if(!intersect_single_rays(left_rays_.greens[l], 
                                      right_rays_.greens[r],
                                      &g_p.back()))
                g_p.pop_back();
        }
    }

    // Return if none found
    if(!g_p.size()) {
        ROS_INFO("No valid green intersections found!");
        return;
    }

    // Remove any duplicates
    for(int i = 0; i < g_p.size(); i++) {
        for(int j  = i+1; j < g_p.size(); j++) {
            if(mag(g_p[i].point - g_p[j].point) < 0.0001) {
                g_p.erase(g_p.begin() + j);
                j--;
            }
        }
    }

    // Repeat for blue

    // Store the Point and the ray ids
    std::vector<LedPoint> b_p;

    // Loop through all blue rays and find all potential points
    for(int l = 0; l < left_rays_.blues.size(); l++) {
        for(int r = 0; r < right_rays_.blues.size(); r++) {
            b_p.push_back(LedPoint());
            b_p.back().ray_id_1 = l;
            b_p.back().ray_id_2 = r;
            // Check intersection, delete if no intersection
            if(!intersect_single_rays(left_rays_.blues[l], 
                                      right_rays_.blues[r],
                                      &b_p.back()))
                b_p.pop_back();
        }
    }

    // Return if none found
    if(!b_p.size()) {
        ROS_INFO("No valid blue intersections found");
        return;
    }

    // Remove any duplicates
    for(int i = 0; i < b_p.size(); i++) {
        for(int j  = i+1; j < b_p.size(); j++) {
            if(mag(b_p[i].point - b_p[j].point) < 0.0001) {
                b_p.erase(b_p.begin() + j);
                j--;
            }
        }
    }

    if(debug_) {
        debug_publish_points(g_p, b_p);
    }

    ROS_INFO("g_size: %d -- b_size: %d", g_p.size(), b_p.size());

    // Find publish the pose of the glove
    finder_.find_pose(g_p, b_p);
}

bool RayIntersector::intersect_single_rays(geometry_msgs::Vector3 left,
                                           geometry_msgs::Vector3 right,
                                           LedPoint *point) {
    // Get ray starting positions
    cv::Point3f left_pos, right_pos;
    left_pos.x = left_cam_tf_.transform.translation.x;
    left_pos.y = left_cam_tf_.transform.translation.y;
    left_pos.z = left_cam_tf_.transform.translation.z;
    right_pos.x = right_cam_tf_.transform.translation.x;
    right_pos.y = right_cam_tf_.transform.translation.y;
    right_pos.z = right_cam_tf_.transform.translation.z;

    // Transform vectors from camera frame to world frame
    tf2::doTransform(left, left, left_cam_tf_);
    tf2::doTransform(right, right, right_cam_tf_);

    // Convert geometry_msg vector to cv vector
    cv::Vec3f l;
    cv::Vec3f r;
    l[0] = left.x; l[1] = left.y; l[2] = left.z;
    r[0] = right.x; r[1] = right.y; r[2] = right.z;

    // Rearranging to matrix equation gives:
    // M * (mu_r, lam, mu_l)' = (left_pos - right_pos)
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
    
    cv::Vec3f scalars = M.inv() * (left_pos - right_pos);
    mu_r = scalars[0];
    lam = scalars[1];
    mu_l = scalars[2];

    // If lambda is less than the allowed tolerance then accept intersection
    if(fabs(lam) > intersection_tol_) {
        ROS_INFO("Bad tolerance");
        return false;
    }

    // Calculate center
    cv::Point3f center = (cv::Vec3f)right_pos + mu_r * r + cr * lam / 2;
    
    // If the center is not within the allowed bounds then ignore
    if(min_x_ > center.x || max_x_ < center.x ||
       min_y_ > center.y || max_y_ < center.y ||
       min_z_ > center.z || max_z_ < center.z) {
        ROS_INFO("Out of bounds: [%0.2f, %0.2f, %0.2f]", 
                 center.x, center.y, center.z);
        return false;
    }
    
    // Store the associated point
    point->point = center;
    return true;
}

void RayIntersector::debug_publish_points(std::vector<LedPoint> g,
                                          std::vector<LedPoint> b) {
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "world";

    trans.transform.rotation.w = 1;

    for(int i = 0; i < g.size(); i++) {
        trans.child_frame_id = "green_" + std::to_string(i);

        trans.transform.translation.x = g[i].point.x;
        trans.transform.translation.y = g[i].point.y;
        trans.transform.translation.z = g[i].point.z;

        tf_broadcaster_.sendTransform(trans);
    }

    for(int i = 0; i < b.size(); i++) {
        trans.child_frame_id = "blue_" + std::to_string(i);

        trans.transform.translation.x = b[i].point.x;
        trans.transform.translation.y = b[i].point.y;
        trans.transform.translation.z = b[i].point.z;

        tf_broadcaster_.sendTransform(trans);
    }
}
