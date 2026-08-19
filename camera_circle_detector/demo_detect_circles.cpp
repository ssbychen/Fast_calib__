#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "CameraCircleCenterDetector.h"

namespace {

std::string defaultOutputPath(const std::string& inputPath) {
  const std::string suffix = "_annotated";
  const std::size_t dot = inputPath.find_last_of('.');
  if (dot == std::string::npos || dot == 0) {
    return inputPath + suffix + ".jpg";
  }
  return inputPath.substr(0, dot) + suffix + inputPath.substr(dot);
}

bool readDouble(const cv::FileStorage& fs, const std::string& key,
                double& value) {
  const cv::FileNode node = fs[key];
  if (node.empty()) {
    return false;
  }
  value = static_cast<double>(node);
  return true;
}

bool readFloat(const cv::FileStorage& fs, const std::string& key, float& value) {
  const cv::FileNode node = fs[key];
  if (node.empty()) {
    return false;
  }
  value = static_cast<float>(node);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: camera_circle_demo <input.jpg> [output.jpg] "
                 "[config.yaml]\n";
    return 1;
  }

  const std::string inputImagePath = argv[1];
  const std::string outputImagePath =
      (argc >= 3) ? argv[2] : defaultOutputPath(inputImagePath);
  const std::string configPath = (argc >= 4) ? argv[3] : "config/qr_params.yaml";

  cv::FileStorage fs(configPath, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Failed to open config file: " << configPath << "\n";
    return 1;
  }

  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0;
  float marker_size = 0.0f, delta_width_qr_center = 0.0f,
        delta_height_qr_center = 0.0f, delta_width_circles = 0.0f,
        delta_height_circles = 0.0f;

  if (!readDouble(fs, "fx", fx) || !readDouble(fs, "fy", fy) ||
      !readDouble(fs, "cx", cx) || !readDouble(fs, "cy", cy) ||
      !readDouble(fs, "k1", k1) || !readDouble(fs, "k2", k2) ||
      !readDouble(fs, "p1", p1) || !readDouble(fs, "p2", p2) ||
      !readFloat(fs, "marker_size", marker_size) ||
      !readFloat(fs, "delta_width_qr_center", delta_width_qr_center) ||
      !readFloat(fs, "delta_height_qr_center", delta_height_qr_center) ||
      !readFloat(fs, "delta_width_circles", delta_width_circles) ||
      !readFloat(fs, "delta_height_circles", delta_height_circles)) {
    std::cerr << "Missing required camera/board parameters in: " << configPath
              << "\n";
    return 1;
  }
  (void)readDouble(fs, "k3", k3);

  cv::Mat image = cv::imread(inputImagePath, cv::IMREAD_COLOR);
  if (image.empty()) {
    std::cerr << "Failed to load image: " << inputImagePath << "\n";
    return 1;
  }

  cv::Mat K =
      (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  cv::Mat D = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);

  CameraCircleCenterDetector detector(marker_size, delta_width_qr_center,
                                      delta_height_qr_center,
                                      delta_width_circles,
                                      delta_height_circles, K, D);

  const CameraCircleCenterDetector::Result result = detector.detect(image);
  if (!result.success) {
    std::cerr << "Circle center detection failed.\n";
    return 1;
  }

  const char* names[4] = {"left-top", "right-top", "left-bottom",
                          "right-bottom"};
  for (int i = 0; i < 4; ++i) {
    std::cout << names[i] << ": (" << result.centers_2d[i].x << ", "
              << result.centers_2d[i].y << ")\n";
  }

  for (int i = 0; i < 4; ++i) {
    const cv::Point2f& p = result.centers_2d[i];
    cv::circle(image, p, 8, cv::Scalar(0, 255, 0), -1);
    cv::putText(image, names[i], p + cv::Point2f(8.0f, -8.0f),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
  }

  if (!cv::imwrite(outputImagePath, image)) {
    std::cerr << "Failed to save annotated image: " << outputImagePath << "\n";
    return 1;
  }

  std::cout << "Annotated image saved to: " << outputImagePath << "\n";
  return 0;
}
