cmake_minimum_required(VERSION 3.1)
project(WebServer)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Where is OpenCV installed?
SET(OpenCV_DIR /home/nickick/OpenCV_installation/installation/OpenCV-master/lib/cmake/opencv4)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

set(SOURCE_FILES 
    main.cpp
)

set(Boost_LIBRARIES
    Boost::system
)

add_executable(
    ${PROJECT_NAME}
    ${SOURCE_FILES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)
