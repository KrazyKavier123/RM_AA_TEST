// ROI dimensions/normalization follow ChenJun rm_auto_aim (MIT); see THIRD_PARTY.md.
#include "rm_aa_vision/classifier.hpp"
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <opencv2/imgproc.hpp>
namespace rm_aa {
Classifier::Classifier(const std::string & model, const std::string & labels, double threshold)
  : net_(cv::dnn::readNetFromONNX(model)), threshold_(threshold) {
  if (threshold < 0 || threshold > 1) throw std::invalid_argument("Classification threshold outside [0,1]");
  std::ifstream input(labels);
  if (!input) throw std::runtime_error("Cannot read label file: " + labels);
  for (std::string line; std::getline(input, line);) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (!line.empty()) labels_.push_back(line);
  }
  if (labels_.empty()) throw std::runtime_error("Empty labels");
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}
cv::Mat Classifier::extract(const cv::Mat & bgr, const Plate & plate) const {
  const int width = plate.size == PlateSize::Small ? 32 : 54;
  // Reference model uses 12px light length in a 28px-high number image.
  const std::vector<cv::Point2f> src{plate.corners[3], plate.corners[0], plate.corners[1], plate.corners[2]};
  const std::vector<cv::Point2f> dst{{0, 19}, {0, 7},
    {static_cast<float>(width - 1), 7}, {static_cast<float>(width - 1), 19}};
  cv::Mat warped, gray, binary;
  cv::warpPerspective(bgr, warped, cv::getPerspectiveTransform(src, dst), {width, 28});
  cv::cvtColor(warped(cv::Rect((width - 20) / 2, 0, 20, 28)), gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  return binary;
}
void Classifier::classify(const cv::Mat & bgr, Plate & plate) {
  const auto binary = extract(bgr, plate);
  // Convert through blobFromImage to float; avoid integer Mat division.
  net_.setInput(cv::dnn::blobFromImage(binary, 1.0 / 255.0));
  const cv::Mat logits = net_.forward().reshape(1, 1);
  if (logits.total() != labels_.size() || logits.type() != CV_32F || !cv::checkRange(logits))
    throw std::runtime_error("Invalid model output shape/type/value");
  double maximum; cv::minMaxLoc(logits, nullptr, &maximum);
  cv::Mat probabilities; cv::exp(logits - maximum, probabilities);
  const double sum = cv::sum(probabilities)[0];
  if (!std::isfinite(sum) || sum <= 0) throw std::runtime_error("Invalid classification probabilities");
  probabilities /= sum;
  cv::Point index; cv::minMaxLoc(probabilities, nullptr, &plate.confidence, nullptr, &index);
  plate.label = labels_.at(index.x);
  plate.classification_valid = plate.confidence >= threshold_ && plate.label != "negative";
}
}
