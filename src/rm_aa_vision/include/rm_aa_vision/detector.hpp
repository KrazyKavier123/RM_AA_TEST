#pragma once
#include "rm_aa_vision/types.hpp"
namespace rm_aa {
struct DetectorConfig {
  Color observed_color{Color::Blue};
  float min_light_length{8}, max_light_width_ratio{0.45F}, max_tilt_deg{35};
  float min_pair_ratio{1.5F}, max_pair_ratio{5.0F}, large_pair_ratio{3.3F};
  float min_length_ratio{0.65F}, max_vertical_ratio{0.6F}, min_color_difference{8};
};
class Detector {
public:
  explicit Detector(DetectorConfig config = {});
  std::vector<Light> find_lights(const cv::Mat & bgr, const cv::Mat & mask) const;
  std::vector<Plate> match(const std::vector<Light> & lights) const;
private:
  DetectorConfig config_;
};
}
