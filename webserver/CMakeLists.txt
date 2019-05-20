cmake_minimum_required(VERSION 3.1)
project(WebServer)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Where is OpenCV installed?
SET(OpenCV_DIR /home/nickick/OpenCV_installation/installation/OpenCV-master/lib/cmake/opencv4)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

set(SOURCE_FILES 
    server_main.cpp
    src/connection_manager.cpp
)

set(CLIENT_SOURCE_FILES
    client_main.cpp
)

set(Boost_LIBRARIES
    Boost::system
    Boost::chrono
)

add_executable(
    ${PROJECT_NAME}
    ${SOURCE_FILES}
)

add_executable(
    WebClient
    ${CLIENT_SOURCE_FILES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

target_link_libraries(
    WebClient
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)
