/*
 * CameraCircleCenterDetector.cpp
 *
 * Standalone, OpenCV-only implementation.
 * See CameraCircleCenterDetector.h for API documentation.
 */

#include "CameraCircleCenterDetector.h"

#include <cmath>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CameraCircleCenterDetector::CameraCircleCenterDetector(
    float           marker_size,
    float           delta_width_qr_center,
    float           delta_height_qr_center,
    float           delta_width_circles,
    float           delta_height_circles,
    const cv::Mat&  cameraMatrix,
    const cv::Mat&  distCoeffs)
    : marker_size_(marker_size),
      delta_width_qr_center_(delta_width_qr_center),
      delta_height_qr_center_(delta_height_qr_center),
      delta_width_circles_(delta_width_circles),
      delta_height_circles_(delta_height_circles),
      cameraMatrix_(cameraMatrix.clone()),
      distCoeffs_(distCoeffs.clone())
{
    // Ensure float precision for the intrinsics used internally
    if (cameraMatrix_.type() != CV_64F)
        cameraMatrix_.convertTo(cameraMatrix_, CV_64F);
    if (distCoeffs_.type() != CV_64F)
        distCoeffs_.convertTo(distCoeffs_, CV_64F);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

cv::Point2f CameraCircleCenterDetector::projectPointDist(const cv::Point3f& pt) const
{
    std::vector<cv::Point3f> pts3d{pt};
    std::vector<cv::Point2f> pts2d(1);
    cv::projectPoints(
        pts3d,
        cv::Mat::zeros(3, 1, CV_64FC1),  // no additional rotation
        cv::Mat::zeros(3, 1, CV_64FC1),  // no additional translation
        cameraMatrix_,
        distCoeffs_,
        pts2d);
    return pts2d[0];
}

// ---------------------------------------------------------------------------
// detect()
// ---------------------------------------------------------------------------

CameraCircleCenterDetector::Result
CameraCircleCenterDetector::detect(const cv::Mat& image) const
{
    Result result;

    if (image.empty()) {
        std::cerr << "[CameraCircleCenterDetector] Input image is empty.\n";
        return result;
    }

    // ------------------------------------------------------------------
    // 1. Build board geometry
    //
    // Board layout (marker indices):
    //   0 (ID 1) -------- 1 (ID 2)
    //      |                 |
    //      |    circle grid  |
    //      |                 |
    //   3 (ID 3) -------- 2 (ID 4)
    //
    // Circle centres in board frame (index -> quadrant):
    //   0: left-top     (-x, +y)
    //   1: right-top    (+x, +y)
    //   2: left-bottom  (-x, -y)
    //   3: right-bottom (+x, -y)
    // ------------------------------------------------------------------

    const float width        = delta_width_qr_center_;
    const float height       = delta_height_qr_center_;
    const float circle_width  = delta_width_circles_  / 2.0f;
    const float circle_height = delta_height_circles_ / 2.0f;

    std::vector<std::vector<cv::Point3f>> boardCorners(4);
    std::vector<cv::Point3f> boardCircleCenters;
    boardCircleCenters.reserve(4);

    for (int i = 0; i < 4; ++i)
    {
        // Sign conventions preserved from qr_detect.hpp
        int x_qr_center = (i % 3) == 0 ? -1 : 1;
        int y_qr_center = (i < 2)      ?  1 : -1;

        float x_center = x_qr_center * width;
        float y_center = y_qr_center * height;

        // Circle centre in board coordinate frame
        boardCircleCenters.push_back(cv::Point3f(
            x_qr_center * circle_width,
            y_qr_center * circle_height,
            0.0f));

        // Four corners of this ArUco marker
        boardCorners[i].reserve(4);
        for (int j = 0; j < 4; ++j)
        {
            int x_qr = (j % 3) == 0 ? -1 : 1;
            int y_qr = (j < 2)      ?  1 : -1;
            boardCorners[i].push_back(cv::Point3f(
                x_center + x_qr * marker_size_ / 2.0f,
                y_center + y_qr * marker_size_ / 2.0f,
                0.0f));
        }
    }

    // Marker ID assignment (same order as qr_detect.hpp):
    //   board index 0 -> ArUco ID 1
    //   board index 1 -> ArUco ID 2
    //   board index 2 -> ArUco ID 4
    //   board index 3 -> ArUco ID 3
    std::vector<int> boardIds{1, 2, 4, 3};

    cv::Ptr<cv::aruco::Board> board =
        cv::aruco::Board::create(boardCorners, dictionary_, boardIds);

    // ------------------------------------------------------------------
    // 2. Detect ArUco markers in the image
    // ------------------------------------------------------------------

    cv::Ptr<cv::aruco::DetectorParameters> params =
        cv::aruco::DetectorParameters::create();

#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    params->doCornerRefinement = true;
#else
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#endif

    std::vector<int>                        ids;
    std::vector<std::vector<cv::Point2f>>   corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, params);

    result.detected_ids = ids;

    if (ids.size() < 3) {
        std::cerr << "[CameraCircleCenterDetector] Only " << ids.size()
                  << " marker(s) detected (need >= 3). Skipping frame.\n";
        return result;
    }

    // ------------------------------------------------------------------
    // 3. Compute initial pose guess from individual markers, then refine
    //    with estimatePoseBoard (mirrors qr_detect.hpp exactly)
    // ------------------------------------------------------------------

    cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0);

    {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
            corners, marker_size_, cameraMatrix_, distCoeffs_, rvecs, tvecs);

        cv::Vec3f rvec_sin(0, 0, 0), rvec_cos(0, 0, 0);
        for (size_t i = 0; i < ids.size(); ++i)
        {
            tvec[0] += tvecs[i][0];
            tvec[1] += tvecs[i][1];
            tvec[2] += tvecs[i][2];
            rvec_sin[0] += static_cast<float>(std::sin(rvecs[i][0]));
            rvec_sin[1] += static_cast<float>(std::sin(rvecs[i][1]));
            rvec_sin[2] += static_cast<float>(std::sin(rvecs[i][2]));
            rvec_cos[0] += static_cast<float>(std::cos(rvecs[i][0]));
            rvec_cos[1] += static_cast<float>(std::cos(rvecs[i][1]));
            rvec_cos[2] += static_cast<float>(std::cos(rvecs[i][2]));
        }

        int n = static_cast<int>(ids.size());
        tvec    /= n;
        rvec_sin /= n;
        rvec_cos /= n;
        rvec[0] = std::atan2(rvec_sin[0], rvec_cos[0]);
        rvec[1] = std::atan2(rvec_sin[1], rvec_cos[1]);
        rvec[2] = std::atan2(rvec_sin[2], rvec_cos[2]);
    }

#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    int valid = cv::aruco::estimatePoseBoard(
        corners, ids, board, cameraMatrix_, distCoeffs_, rvec, tvec);
#else
    int valid = cv::aruco::estimatePoseBoard(
        corners, ids, board, cameraMatrix_, distCoeffs_, rvec, tvec, true);
#endif

    if (valid <= 0) {
        std::cerr << "[CameraCircleCenterDetector] estimatePoseBoard failed.\n";
        return result;
    }

    // ------------------------------------------------------------------
    // 4. Build the 3x4 board-to-camera transform
    // ------------------------------------------------------------------

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    R.convertTo(R, CV_32F);

    cv::Mat board_transform = cv::Mat::eye(3, 4, CV_32F);
    R.copyTo(board_transform.rowRange(0, 3).colRange(0, 3));
    board_transform.at<float>(0, 3) = static_cast<float>(tvec[0]);
    board_transform.at<float>(1, 3) = static_cast<float>(tvec[1]);
    board_transform.at<float>(2, 3) = static_cast<float>(tvec[2]);

    // ------------------------------------------------------------------
    // 5. Transform the four circle centres to camera frame and project
    // ------------------------------------------------------------------

    for (int i = 0; i < 4; ++i)
    {
        cv::Mat pt_board = cv::Mat::zeros(4, 1, CV_32F);
        pt_board.at<float>(0) = boardCircleCenters[i].x;
        pt_board.at<float>(1) = boardCircleCenters[i].y;
        pt_board.at<float>(2) = boardCircleCenters[i].z;
        pt_board.at<float>(3) = 1.0f;

        cv::Mat pt_cam = board_transform * pt_board;

        result.centers_3d[i] = cv::Point3f(
            pt_cam.at<float>(0),
            pt_cam.at<float>(1),
            pt_cam.at<float>(2));

        result.centers_2d[i] = projectPointDist(result.centers_3d[i]);
    }

    result.success = true;
    return result;
}
