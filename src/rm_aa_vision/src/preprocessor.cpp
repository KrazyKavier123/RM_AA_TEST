#include "rm_aa_vision/preprocessor.hpp"
#include <stdexcept>
#include <opencv2/imgproc.hpp>
namespace rm_aa {
Preprocessor::Preprocessor(PreprocessConfig config) : config_(config) {
  if (config.brightness_threshold < 0 || config.brightness_threshold > 255 ||
      config.opening_radius < 0 || config.opening_radius > 5)
    throw std::invalid_argument("Invalid preprocessing thresholds");
}
cv::Mat Preprocessor::run(const cv::Mat & bgr) const {
  if (bgr.empty() || bgr.type() != CV_8UC3) throw std::invalid_argument("Expected BGR8 image");
  cv::Mat gray, mask;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, mask, config_.brightness_threshold, 255, cv::THRESH_BINARY);
  if (config_.opening_radius > 0) {
    const int k = 2 * config_.opening_radius + 1;
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k}));
  }
  return mask;
}
}
