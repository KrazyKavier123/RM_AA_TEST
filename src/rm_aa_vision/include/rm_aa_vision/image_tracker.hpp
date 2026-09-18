#pragma once
#include "rm_aa_vision/types.hpp"
#include <opencv2/video/tracking.hpp>
namespace rm_aa {
// Diagnostic image-space association only. No vehicle/ballistic model or control interface.
struct ImageTrack {
  int id{};
  PlateSize size{PlateSize::Small};
  std::string label;
  cv::KalmanFilter filter;
  cv::Point2f position{}, velocity_px_s{};
  double last_seen{};
  bool observed{false};
};
class ImageTracker {
public:
  ImageTracker(double association_px = 80, double timeout_s = 0.3);
  std::vector<ImageTrack> update(const std::vector<Plate> & plates, double time_s);
  void reset();
  int reset_count() const { return reset_count_; }
private:
  std::vector<ImageTrack> tracks_;
  double association_px_, timeout_s_;
  std::optional<double> last_time_;
  int next_id_{1}, reset_count_{0};
};
}
