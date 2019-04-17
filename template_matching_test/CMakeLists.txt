cmake_minimum_required(VERSION 3.1)
project(TemplateMatcher)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Where is OpenCV installed?
SET(OpenCV_DIR /home/nickick/OpenCV_installation/installation/OpenCV-master/lib/cmake/opencv4)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

set(SOURCE_FILES 
    main.cpp
    src/camera.cpp
)

add_executable(
    ${PROJECT_NAME}
    ${SOURCE_FILES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)
