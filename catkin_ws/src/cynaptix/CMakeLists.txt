cmake_minimum_required(VERSION 2.8.3)
project(cynaptix)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf
  image_transport
  cv_bridge  
)

# System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

find_package(Qt5 COMPONENTS Core Widgets PrintSupport)

# Catkin setup
catkin_package()

include_directories(
  include
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5PrintSupport_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(glove_tracking_node
  glove_tracking_node.cpp
  src/object_tracker.cpp
  src/pose_calculator.cpp
  src/pose_calculator_3d.cpp
  ${QT_RESOURCES_CPP} 
  ${QT_FORMS_HPP} 
  ${QT_MOC_HPP}
)

add_executable(led_detector_node
  led_detector_node.cpp
  src/led_detector/led_detector.cpp
)


add_dependencies(glove_tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(led_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  glove_tracking_node
  Qt5::Core
  Qt5::Widgets
  Qt5::PrintSupport
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  led_detector_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

install(TARGETS 
  glove_tracking_node 
  led_detector_node
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

install(PROGRAMS scripts/camera_exposure_setup.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
