/*
 * CameraCircleCenterDetector.h
 *
 * Standalone, OpenCV-only interface for detecting the four camera-side circle
 * centers from a single image using ArUco markers.
 *
 * No ROS, no PCL dependencies.
 *
 * Output order (fixed):
 *   index 0 -> left-top
 *   index 1 -> right-top
 *   index 2 -> left-bottom
 *   index 3 -> right-bottom
 *
 * The ordering follows the board coordinate convention from Fast_calib__:
 *   index 0: (-x, +y)
 *   index 1: (+x, +y)
 *   index 2: (-x, -y)
 *   index 3: (+x, -y)
 */

#ifndef CAMERA_CIRCLE_CENTER_DETECTOR_H
#define CAMERA_CIRCLE_CENTER_DETECTOR_H

#include <array>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

class CameraCircleCenterDetector
{
public:
    /**
     * @brief Detection result.
     *
     * centers_3d[i] - 3-D position in camera frame (metres)
     * centers_2d[i] - projected pixel coordinate
     *
     * Index order: 0=left-top, 1=right-top, 2=left-bottom, 3=right-bottom
     */
    struct Result
    {
        std::array<cv::Point3f, 4> centers_3d;
        std::array<cv::Point2f, 4> centers_2d;
        std::vector<int>           detected_ids;
        bool                       success = false;
    };

    /**
     * @brief Construct the detector.
     *
     * @param marker_size            Side length of each ArUco marker (metres).
     * @param delta_width_qr_center  Half distance between marker centres in X (metres).
     * @param delta_height_qr_center Half distance between marker centres in Y (metres).
     * @param delta_width_circles    Distance between circle centres in X (metres).
     * @param delta_height_circles   Distance between circle centres in Y (metres).
     * @param cameraMatrix           3x3 camera intrinsic matrix (CV_64F or CV_32F).
     * @param distCoeffs             Distortion coefficients (1x4, 1x5, or 1x8).
     */
    CameraCircleCenterDetector(float           marker_size,
                               float           delta_width_qr_center,
                               float           delta_height_qr_center,
                               float           delta_width_circles,
                               float           delta_height_circles,
                               const cv::Mat&  cameraMatrix,
                               const cv::Mat&  distCoeffs);

    /**
     * @brief Detect the four circle centres in a single image.
     *
     * @param image  Input image (colour or grayscale).
     * @return       Result struct; check result.success before using values.
     */
    Result detect(const cv::Mat& image) const;

private:
    float marker_size_;
    float delta_width_qr_center_;
    float delta_height_qr_center_;
    float delta_width_circles_;
    float delta_height_circles_;

    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    /**
     * @brief Project a 3-D camera-frame point to pixel coordinates
     *        with lens distortion.
     */
    cv::Point2f projectPointDist(const cv::Point3f& pt) const;
};

#endif // CAMERA_CIRCLE_CENTER_DETECTOR_H
