#include "rm_aa_vision/detector.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <opencv2/imgproc.hpp>
namespace rm_aa {
Detector::Detector(DetectorConfig config) : config_(config) {
  if (config.min_light_length <= 0 || config.min_length_ratio <= 0 ||
      config.min_length_ratio > 1 || config.max_light_width_ratio <= 0 ||
      config.min_pair_ratio <= 0 || config.large_pair_ratio <= config.min_pair_ratio ||
      config.large_pair_ratio >= config.max_pair_ratio)
    throw std::invalid_argument("Invalid light/pair geometry settings");
}
std::vector<Light> Detector::find_lights(const cv::Mat & bgr, const cv::Mat & mask) const {
  if (bgr.type() != CV_8UC3 || mask.type() != CV_8UC1 || bgr.size() != mask.size())
    throw std::invalid_argument("BGR and binary mask sizes/types do not match");
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<Light> result;
  for (const auto & contour : contours) {
    if (contour.size() < 4) continue;
    const auto box = cv::minAreaRect(contour);
    cv::Point2f p[4]; box.points(p);
    auto axis = p[1] - p[0];
    if (cv::norm(p[2] - p[1]) > cv::norm(axis)) axis = p[2] - p[1];
    const float length = static_cast<float>(cv::norm(axis));
    const float width = std::min(box.size.width, box.size.height);
    if (length < config_.min_light_length || width / length > config_.max_light_width_ratio)
      continue;
    if (axis.y < 0) axis = -axis;
    if (std::abs(std::atan2(axis.x, axis.y)) * 180 / CV_PI > config_.max_tilt_deg) continue;
    const auto roi = cv::boundingRect(contour) & cv::Rect(0, 0, bgr.cols, bgr.rows);
    cv::Mat roi_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    std::vector<cv::Point> local;
    for (const auto & point : contour) local.emplace_back(point - roi.tl());
    cv::drawContours(roi_mask, std::vector<std::vector<cv::Point>>{local}, -1, 255, cv::FILLED);
    const auto mean = cv::mean(bgr(roi), roi_mask);
    if (std::abs(mean[2] - mean[0]) < config_.min_color_difference) continue;
    const Color color = mean[2] > mean[0] ? Color::Red : Color::Blue;
    if (color != config_.observed_color) continue;
    result.push_back({box.center - axis * 0.5F, box.center + axis * 0.5F,
                      box.center, length, width, color});
  }
  std::sort(result.begin(), result.end(), [](const Light & a, const Light & b) {
    return a.center.x < b.center.x;
  });
  return result;
}
std::vector<Plate> Detector::match(const std::vector<Light> & lights) const {
  std::vector<Plate> result;
  for (size_t i = 0; i < lights.size(); ++i) for (size_t j = i + 1; j < lights.size(); ++j) {
    const auto & left = lights[i]; const auto & right = lights[j];
    if (left.color != right.color || left.center.x >= right.center.x) continue;
    const float length = (left.length + right.length) * 0.5F;
    if (length <= 0 || std::min(left.length, right.length) /
      std::max(left.length, right.length) < config_.min_length_ratio) continue;
    const float ratio = (right.center.x - left.center.x) / length;
    if (ratio < config_.min_pair_ratio || ratio > config_.max_pair_ratio ||
        std::abs(right.center.y - left.center.y) / length > config_.max_vertical_ratio) continue;
    bool contains_other = false;
    for (size_t k = i + 1; k < j; ++k) {
      if (std::abs(lights[k].center.y - (left.center.y + right.center.y) * 0.5F) < length)
        contains_other = true;
    }
    if (contains_other) continue;
    Plate plate;
    plate.corners = {left.top, right.top, right.bottom, left.bottom};
    plate.center = (left.center + right.center) * 0.5F;
    plate.size = ratio >= config_.large_pair_ratio ? PlateSize::Large : PlateSize::Small;
    plate.color = left.color;
    result.push_back(plate);
  }
  return result;
}
}
