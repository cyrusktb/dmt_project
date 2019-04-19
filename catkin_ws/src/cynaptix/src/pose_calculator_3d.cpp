#include "pose_calculator_3d.hpp"

PoseCalculator3d::PoseCalculator3d(const ros::NodeHandle& nh) {
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    image_transport_ = new image_transport::ImageTransport(nh);

    // Setup left camera intrinsic matrix listener
    left_intrinsic_(0, 1) = 0;
    left_intrinsic_(1, 0) = 0;
    left_intrinsic_(2, 0) = 0;
    left_intrinsic_(2, 1) = 0;
    left_intrinsic_(2, 2) = 1;
    left_cam_sub_ = image_transport_->subscribeCamera(
        "/left_camera/camera", 1, 
        &PoseCalculator3d::left_camera_info_callback, this
    );

    // Setup right camera intrinsic matrix listener
    right_intrinsic_(0, 1) = 0;
    right_intrinsic_(1, 0) = 0;
    right_intrinsic_(2, 0) = 0;
    right_intrinsic_(2, 1) = 0;
    right_intrinsic_(2, 2) = 1;
    right_cam_sub_ = image_transport_->subscribeCamera(
        "/right_camera/camera", 1, 
        &PoseCalculator3d::right_camera_info_callback, this
    );
}

PoseCalculator3d::~PoseCalculator3d() {
    // dtor
}

void PoseCalculator3d::left_camera_info_callback(
            const sensor_msgs::ImageConstPtr& img, 
            const sensor_msgs::CameraInfoConstPtr& msg) {
    left_intrinsic_(0, 0) = msg->K[0];
    left_intrinsic_(0, 2) = msg->K[2];
    left_intrinsic_(1, 1) = msg->K[4];
    left_intrinsic_(1, 2) = msg->K[5];
}

void PoseCalculator3d::right_camera_info_callback(
            const sensor_msgs::ImageConstPtr& img, 
            const sensor_msgs::CameraInfoConstPtr& msg) {
    right_intrinsic_(0, 0) = msg->K[0];
    right_intrinsic_(0, 2) = msg->K[2];
    right_intrinsic_(1, 1) = msg->K[4];
    right_intrinsic_(1, 2) = msg->K[5];
}

void PoseCalculator3d::calculate_and_publish_pose() {
    if(!left_objects_.size()) return;
    // Get pose of cameras
    try {
        left_cam_tf_ = tf_buffer_.lookupTransform("left_camera_frame",
                                                 "world", ros::Time(0));
        right_cam_tf_ = tf_buffer_.lookupTransform("right_camera_frame",
                                                  "world", ros::Time(0));
    }
    catch(tf2::TransformException &ex) {
        ROS_ERROR("Failed to get camera transforms:\n%s", ex.what());
        return;
    }
    // Get pose for each pair of objects and take the average
    Object left, right;
    left = left_objects_[0];
    right = right_objects_[0];

    // Intersect the rays for left and right
    cv::Point3f intersection_points[4];
    float distances[4];
    cv::Vec3f left_rays[4];
    cv::Vec3f right_rays[4];

    left_rays[0] = get_ray(cv::Point2f(left.top_left.x(), 
                                       left.top_left.y()), 
                           true);
    right_rays[0]= get_ray(cv::Point2f(right.top_left.x(), 
                                       right.top_left.y()), 
                           false);
    left_rays[1] = get_ray(cv::Point2f(left.top_right.x(), 
                                       left.top_right.y()), 
                           true);
    right_rays[1]= get_ray(cv::Point2f(right.top_right.x(), 
                                       right.top_right.y()), 
                           false);
    left_rays[2] = get_ray(cv::Point2f(left.bot_right.x(), 
                                       left.bot_right.y()), 
                           true);
    right_rays[2]= get_ray(cv::Point2f(right.bot_right.x(), 
                                       right.bot_right.y()), 
                           false);
    left_rays[3] = get_ray(cv::Point2f(left.bot_left.x(), 
                                       left.bot_left.y()), 
                           true);
    right_rays[3]= get_ray(cv::Point2f(right.bot_left.x(), 
                                       right.bot_left.y()), 
                           false);

    for(short i = 0; i < 4; i++) {
        cv::Point3f p;
        intersect_rays(left_rays[i],
                       right_rays[i],
                       &p,
                       &(distances[i]));

        transform_stamped_.header.stamp = ros::Time::now();
        transform_stamped_.header.frame_id = "world";
        transform_stamped_.child_frame_id = "object_" 
                                          + std::to_string(left.id)
                                          + "_" + std::to_string(i);
        
        transform_stamped_.transform.translation.x = p.x;
        transform_stamped_.transform.translation.y = p.y;
        transform_stamped_.transform.translation.z = p.z;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);

        transform_stamped_.transform.rotation.x = q.x();
        transform_stamped_.transform.rotation.y = q.y();
        transform_stamped_.transform.rotation.z = q.z();
        transform_stamped_.transform.rotation.w = q.w();

        tf_br_.sendTransform(transform_stamped_);
    }
}

void PoseCalculator3d::add_object_pair(Object left, Object right) {
    left_objects_.push_back(left);
    right_objects_.push_back(right);
}

cv::Vec3f PoseCalculator3d::get_ray(cv::Point2f p, bool is_left_camera) {
    // Convert to homogenous
    cv::Matx31f hom_p(p.x, p.y, 1);

    // Multiply by the inverse of the intrinsic matrix
    if(is_left_camera)
        hom_p = left_intrinsic_.inv() * hom_p;
    else
        hom_p = right_intrinsic_.inv() * hom_p;

    // Normalise
    cv::Vec3f dir(hom_p(0), hom_p(1), hom_p(2));
    dir /= cv::norm(dir);

    return dir;
}

void PoseCalculator3d::intersect_rays(cv::Vec3f l, 
                                      cv::Vec3f r, 
                                      cv::Point3f *center,
                                      float *distance) {
    cv::Point3f left_pos, right_pos;
    left_pos.x = left_cam_tf_.transform.translation.x;
    left_pos.y = left_cam_tf_.transform.translation.y;
    left_pos.z = left_cam_tf_.transform.translation.z;
    right_pos.x = right_cam_tf_.transform.translation.x;
    right_pos.y = right_cam_tf_.transform.translation.y;
    right_pos.z = right_cam_tf_.transform.translation.z;
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
    
    // Calculate center
    
    *center = (cv::Vec3f)right_pos + mu_r * r + cr * lam / 2;
    *distance = lam;
}
