#include "object_tracker.hpp"

ObjectTracker::ObjectTracker(ros::NodeHandle *nh, 
                             std::string object_channel) {
    nh->subscribe(object_channel, 1, 
                  &ObjectTracker::objects_detected_callback, this);
}

ObjectTracker::~ObjectTracker() {
    // dtor
}

std::vector<Object> ObjectTracker::get_most_recent_objects() {
    std::vector<Object> vec;
    if(object_ids_.size() == 0) {
        return vec;
    }
    auto best_time = objects_[object_ids_[0]].time_last_detected;
    for(int i = 0; i < object_ids_.size(); i++) {
        int id = object_ids_[i];
        if(objects_[id].time_last_detected > best_time) {
            best_time = objects_[id].time_last_detected;
            vec.clear();
        }
        if(objects_[id].time_last_detected == best_time)
            vec.push_back(objects_[id]);
    }
    return vec;
}

void ObjectTracker::objects_detected_callback(
            const std_msgs::Float32MultiArray::ConstPtr& msg) {
    const std::vector<float>& data = msg->data;

    // Store the time that the message arrived
    auto time_arrived = std::chrono::system_clock::now();

    // Count in 12s as there are 12 pieces of information per object
    for(unsigned int i = 0; i < data.size(); i+=12) {
        // Get which object it is
        int id = (int)data[i];

        // Add the id to the vector if we haven't seen it yet
        bool id_exists = false;
        for(int i = 0; i < object_ids_.size(); i++) {
            if(object_ids_[i] == id) {
                id_exists = true;
                break;
            }
        }
        if(!id_exists) object_ids_.push_back(id);

        objects_[id].id = id;

        // Read width and height
        objects_[id].width = data[i+1];
        objects_[id].height = data[i+2];

        // Find the corners
        QTransform qtHomography(data[i+3], data[i+4] , data[i+5],
                                data[i+6], data[i+7] , data[i+8],
                                data[i+9], data[i+10], data[i+11]);

        objects_[id].top_left = qtHomography.map(QPointF(0,0));
        objects_[id].top_right = qtHomography.map(QPointF(data[i+1], 0));
        objects_[id].bot_left = qtHomography.map(QPointF(0, data[i+2]));
        objects_[id].bot_right = qtHomography.map(QPointF(data[i+1], 
                                                          data[i+2]));
        // Update the time count last
        objects_[id].time_last_detected = time_arrived;
    }
}
