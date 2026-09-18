#pragma once
#include "rm_aa_vision/types.hpp"
#include <opencv2/dnn.hpp>
namespace rm_aa {
class Classifier {
public:
  Classifier(const std::string & model, const std::string & labels, double threshold);
  cv::Mat extract(const cv::Mat & bgr, const Plate & plate) const;
  void classify(const cv::Mat & bgr, Plate & plate);
private:
  cv::dnn::Net net_;
  std::vector<std::string> labels_;
  double threshold_;
};
}
