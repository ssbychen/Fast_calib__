/*
 * example_main.cpp
 *
 * Minimal usage example for CameraCircleCenterDetector.
 *
 * Build:
 *   mkdir build && cd build
 *   cmake ..
 *   make
 *
 * Run:
 *   ./detect_circles test.jpg
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include "CameraCircleCenterDetector.h"

int main(int argc, char** argv)
{
    const char* imagePath = (argc > 1) ? argv[1] : "test.jpg";

    cv::Mat img = cv::imread(imagePath);
    if (img.empty())
    {
        std::cerr << "Failed to load image: " << imagePath << "\n";
        return 1;
    }

    // ------------------------------------------------------------------
    // Replace these values with your actual camera calibration
    // ------------------------------------------------------------------
    cv::Mat K = (cv::Mat_<double>(3, 3)
        << 1215.31801774424,    0.0,              1047.86571859677,
              0.0,           1214.72961288138,     745.068353101898,
              0.0,              0.0,                 1.0);

    cv::Mat D = (cv::Mat_<double>(1, 5)
        << -0.33574781188503,
            0.10996870793601,
            0.000157303079833973,
            0.000544930726278493,
            0.0);

    // ------------------------------------------------------------------
    // Replace these values with your actual calibration board geometry
    // (all units in metres)
    // ------------------------------------------------------------------
    const float marker_size            = 0.2f;   // ArUco marker side length
    const float delta_width_qr_center  = 0.55f;  // half-distance between marker centres in X
    const float delta_height_qr_center = 0.35f;  // half-distance between marker centres in Y
    const float delta_width_circles    = 0.5f;   // distance between circle centres in X
    const float delta_height_circles   = 0.4f;   // distance between circle centres in Y

    CameraCircleCenterDetector detector(
        marker_size,
        delta_width_qr_center,
        delta_height_qr_center,
        delta_width_circles,
        delta_height_circles,
        K, D);

    CameraCircleCenterDetector::Result result = detector.detect(img);

    if (!result.success)
    {
        std::cerr << "Detection failed (detected "
                  << result.detected_ids.size() << " markers).\n";
        return 1;
    }

    // Output order: 0=left-top, 1=right-top, 2=left-bottom, 3=right-bottom
    const char* labels[] = { "Left-Top    ", "Right-Top   ",
                              "Left-Bottom ", "Right-Bottom" };
    for (int i = 0; i < 4; ++i)
    {
        std::cout << labels[i]
                  << "  2D: " << result.centers_2d[i]
                  << "  3D: " << result.centers_3d[i]
                  << "\n";
    }

    return 0;
}
