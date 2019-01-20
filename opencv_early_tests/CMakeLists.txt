cmake_minimum_required(VERSION 3.1)
project(DisplayImage)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

SET(OpenCV_DIR /home/nickick/OpenCV_installation/installation/OpenCV-master/lib/cmake/opencv4)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

set(SOURCE_FILES 
    main.cpp
    src/led_tracker.cpp
)

add_executable(DisplayImage
    ${SOURCE_FILES}
)

target_link_libraries(
    DisplayImage
    ${OpenCV_LIBS}
)
