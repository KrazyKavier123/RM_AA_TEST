#pragma once
#include <opencv2/core.hpp>
namespace rm_aa {
struct PreprocessConfig { int brightness_threshold{100}; int opening_radius{0}; };
class Preprocessor {
public:
  explicit Preprocessor(PreprocessConfig config = {});
  cv::Mat run(const cv::Mat & bgr) const;
private:
  PreprocessConfig config_;
};
}
