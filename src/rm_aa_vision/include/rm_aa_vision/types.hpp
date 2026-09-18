#pragma once
#include <array>
#include <optional>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace rm_aa {
enum class PlateSize { Small, Large };
enum class Color { Red, Blue };
struct Light {
  cv::Point2f top, bottom, center;
  float length{}, width{};
  Color color{Color::Blue};
};
struct Plate {
  // Clockwise: top-left, top-right, bottom-right, bottom-left. Light endpoints.
  std::array<cv::Point2f, 4> corners{};
  PlateSize size{PlateSize::Small};
  Color color{Color::Blue};
  cv::Point2f center{};
  std::string label{"unclassified"};
  double confidence{};
  bool classification_valid{false};
};
struct PoseObservation {
  cv::Vec3d rotation{}, translation{};
  double reprojection_rmse_px{};
  double bearing_yaw_rad{}, bearing_pitch_rad{};
};
struct Calibration {
  cv::Matx33d matrix{1000, 0, 640, 0, 1000, 360, 0, 0, 1};
  std::vector<double> distortion{0, 0, 0, 0, 0};
  int width{1280}, height{720};
  bool calibrated{false};
};
struct Dimensions { double small_width_m{0.135}, large_width_m{0.225}, height_m{0.055}; };
inline const char * size_name(PlateSize s) { return s == PlateSize::Small ? "small" : "large"; }
inline const char * color_name(Color c) { return c == Color::Red ? "red" : "blue"; }
}
