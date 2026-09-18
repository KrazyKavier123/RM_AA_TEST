#pragma once
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <cstdint>
namespace rm_aa {
struct SourceConfig {
  std::string mode{"synthetic"}, device{"0"}, video_path;
  int width{1280}, height{720};
  double fps{60};
  bool loop_video{false};
};
cv::Mat synthetic_frame(int width, int height, double time_s);
class Source {
public:
  explicit Source(SourceConfig config);
  bool read(cv::Mat & image);
  double nominal_fps() const;
  bool realtime() const { return config_.mode == "camera"; }
private:
  SourceConfig config_;
  cv::VideoCapture capture_;
  uint64_t index_{0};
};
}
