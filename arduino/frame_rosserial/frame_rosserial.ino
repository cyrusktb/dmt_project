// ROS stuff will only compile if the libraries etc. are installed in the correct locations.
// Commenting this out will result in none of the ROS dependencies being included in the compile
//#define USE_ROS

#ifdef USE_ROS
#include <ros.h>
#include <cynaptix/FrameMeasuredData.h>
#include <cynaptix/FrameTarget.h>
#endif // USE_ROS

/*
 * ##################
 * NORMAL DEFINITIONS
 * ##################
 */

float targetX;
float targetY;
float targetZ;
float targetRot;
float targetGrab;

/*
 * ###############
 * ROS DEFINITIONS
 * ###############
 */
#ifdef USE_ROS
// Uncomment this to average all readings since the last message when sending data to frame
// Comment this to send only the latest measured position when sending data to the frame
#define AVERAGE_READINGS

// Communication frequency with the server (Hz)
#define PUBLISH_FREQ 10

// Callback for when we receive new callbacks
void frame_target_callback(const cynaptix::FrameTarget& msg);

// Node handle to interface with ROS
ros::NodeHandle nh;

// Data we have measured message
cynaptix::FrameMeasuredData measured_data_msg;

// Target positions message
cynaptix::FrameTarget target_msg;

// Publisher
ros::Publisher pub("frame_measured_data", &measured_data_msg);

// Subscriber
ros::Subscriber<cynaptix::FrameTarget> sub("frame_target", frame_target_callback, 1);

// Timer variable to control publish rate
unsigned long ros_timer;

// Variable used to count the number of messages since the previous send when averaging
#ifdef AVERAGE_READINGS
int average_counter;
#endif // AVERAGE_READINGS

#endif // USE_ROS

// Function forward declarations
void setup_ros();

float update_x_pos();
float update_y_pos();
float update_z_pos();
float update_yaw_pos();
float update_grab_pos();

float measure_x_trq();
float measure_y_trq();
float measure_z_trq();
float measure_yaw_trq();
float measure_grab_trq();

void update_ros(float x_pos, float y_pos, float z_pos, float yaw_pos, float grab_pos,
                float x_trq, float y_trq, float z_trq, float yaw_trq, float grab_trq);

void setup() {
  // Setup other shit, inc. home axis if we're implementing that
  // Setup ROS - do this last
  setup_ros();
}

void loop() {
  // update the motor positions
  float x_pos = update_x_pos();
  float y_pos = update_y_pos();
  float z_pos = update_z_pos();
  float yaw_pos = update_yaw_pos();
  float grab_pos = update_grab_pos();

  // measure the currents
  float x_trq = measure_x_trq();
  float y_trq = measure_y_trq();
  float z_trq = measure_z_trq();
  float yaw_trq = measure_yaw_trq();
  float grab_trq = measure_grab_trq();

  // Update ROS
  update_ros(x_pos, y_pos, z_pos, yaw_pos, grab_pos, 
             x_trq, y_trq, z_trq, yaw_trq, grab_trq);
}

void setup_ros() {
#ifdef USE_ROS
  // This is all that goes here for now
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  
#ifdef AVERAGE_READINGS
  measured_data_msg.x_pos = 0;
  measured_data_msg.y_pos = 0;
  measured_data_msg.z_pos = 0;
  measured_data_msg.theta_pos = 0;
  measured_data_msg.grabber_pos = 0;
  
  measured_data_msg.x_torque = 0;
  measured_data_msg.y_torque = 0;
  measured_data_msg.z_torque = 0;
  measured_data_msg.theta_torque = 0;
  measured_data_msg.grabber_torque = 0;

  average_counter = 0;
#endif // AVERAGE_READINGS

  ros_timer = millis();
#endif // USE_ROS
}

void update_ros(float x_pos, float y_pos, float z_pos, float yaw_pos, float grab_pos,
                float x_trq, float y_trq, float z_trq, float yaw_trq, float grab_trq) {
#ifdef USE_ROS
#ifdef AVERAGE_READINGS
  measured_data_msg.x_pos += x_pos;
  measured_data_msg.y_pos += y_pos;
  measured_data_msg.z_pos += z_pos;
  measured_data_msg.theta_pos += yaw_pos;
  measured_data_msg.grabber_pos += grab_pos;
  
  measured_data_msg.x_torque += x_pos;
  measured_data_msg.y_torque += y_pos;
  measured_data_msg.z_torque += z_pos;
  measured_data_msg.theta_torque += yaw_pos;
  measured_data_msg.grabber_torque += grab_pos;

  average_counter++;
#else
  measured_data_msg.x_pos = x_pos;
  measured_data_msg.y_pos = y_pos;
  measured_data_msg.z_pos = z_pos;
  measured_data_msg.theta_pos = yaw_pos;
  measured_data_msg.grabber_pos = grab_pos;
  
  measured_data_msg.x_torque = x_trq;
  measured_data_msg.y_torque = y_trq;
  measured_data_msg.z_torque = z_trq;
  measured_data_msg.theta_torque = yaw_trq;
  measured_data_msg.grabber_torque = grab_trq;
#endif // AVERAGE_READINGS

  if(millis() - ros_timer > 1.0f / PUBLISH_FREQ) {
    ros_timer = millis();
#ifdef AVERAGE_READINGS
    measured_data_msg.x_pos /= (float)average_counter;
    measured_data_msg.y_pos /= (float)average_counter;
    measured_data_msg.z_pos /= (float)average_counter;
    measured_data_msg.theta_pos /= (float)average_counter;
    measured_data_msg.grabber_pos /= (float)average_counter;
    
    measured_data_msg.x_torque /= (float)average_counter;
    measured_data_msg.y_torque /= (float)average_counter;
    measured_data_msg.z_torque /= (float)average_counter;
    measured_data_msg.theta_torque /= (float)average_counter;
    measured_data_msg.grabber_torque /= (float)average_counter;
#endif // AVERAGE_READINGS
    pub.publish(&measured_data_msg);

#ifdef AVERAGE_READINGS
    measured_data_msg.x_pos = 0;
    measured_data_msg.y_pos = 0;
    measured_data_msg.z_pos = 0;
    measured_data_msg.theta_pos = 0;
    measured_data_msg.grabber_pos = 0;
    
    measured_data_msg.x_torque = 0;
    measured_data_msg.y_torque = 0;
    measured_data_msg.z_torque = 0;
    measured_data_msg.theta_torque = 0;
    measured_data_msg.grabber_torque = 0;

    average_counter = 0;
#endif // AVERAGE_READINGS
  }

  // Check for new messages
  nh.spinOnce();
#endif // USE_ROS
}

#ifdef USE_ROS
void frame_target_callback(const cynaptix::FrameTarget& msg) {
  targetX = msg.x_pos;
  targetY = msg.y_pos;
  targetZ = msg.z_pos;
  targetRot = msg.theta_pos;
  targetGrab = msg.grabber_pos;
}
#endif // USE_ROS

float update_x_pos() {
  return 0;
}
float update_y_pos() {
  return 0;
}
float update_z_pos() {
  return 0;
}
float update_yaw_pos() {
  return 0;
}
float update_grab_pos() {
  return 0;
}

float measure_x_trq() {
  return 0;
}
float measure_y_trq() {
  return 0;
}
float measure_z_trq() {
  return 0;
}
float measure_yaw_trq() {
  return 0;
}
float measure_grab_trq() {
  return 0;
}
