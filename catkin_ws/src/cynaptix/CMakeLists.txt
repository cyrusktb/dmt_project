cmake_minimum_required(VERSION 2.8.3)
project(cynaptix)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge  
  message_generation
  visualization_msgs
)

catkin_python_setup()

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  LedRayArray.msg
  GloveMeasuredData.msg
  GloveTarget.msg
  FrameMeasuredData.msg
  FrameTarget.msg
)

# Generate the added messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Catkin setup
catkin_package()

include_directories(
  include
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(led_detector_node
  led_detector_node.cpp
  src/led_detector/led_detector.cpp
  src/led_detector/point_picker.cpp
)

add_executable(glove_position_node
  glove_position_node.cpp
  src/glove_position/ray_intersector.cpp
  src/glove_position/pose_finder.cpp
  src/glove_position/maths.cpp
)

add_dependencies(led_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(glove_position_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  led_detector_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  glove_position_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

install(TARGETS 
  led_detector_node
  glove_position_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY objects
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY camera
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/camera_exposure_setup.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/glove_connection_point.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/frame_connection_point.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
