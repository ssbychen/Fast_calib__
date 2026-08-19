cmake_minimum_required(VERSION 3.5)
project(camera_circle_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

# --- Library ---
add_library(camera_circle_detector
    CameraCircleCenterDetector.cpp
)

target_include_directories(camera_circle_detector PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_circle_detector PUBLIC
    opencv_core
    opencv_imgproc
    opencv_calib3d
    opencv_aruco
)

# --- Example executable ---
add_executable(detect_circles example_main.cpp)

target_include_directories(detect_circles PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(detect_circles PRIVATE
    camera_circle_detector
    opencv_highgui   # for cv::imread / cv::imshow
)
