#pragma once
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <cstdint>
#include <memory>
#include <vector>
namespace rm_aa {
struct SourceConfig {
  std::string mode{"video"}, device{"0"}, video_path;
  int width{1280}, height{720};
  double fps{60};
  bool loop_video{false};
  std::string camera_backend{"hikrobot"}, serial_number;
  double exposure_us{5000.0}, gain_db{0.0};
  int grab_timeout_ms{1000};
};
class Source {
public:
  explicit Source(SourceConfig config);
  ~Source();
  Source(const Source &) = delete;
  Source & operator=(const Source &) = delete;
  // true + empty image means a temporary SDK timeout; false means end of input.
  bool read(cv::Mat & image);
  double nominal_fps() const;
  bool realtime() const { return config_.mode == "camera"; }
private:
  struct HikCamera;
  std::unique_ptr<HikCamera> hik_camera_;
  SourceConfig config_;
  cv::VideoCapture capture_;
  std::vector<std::string> videos_;
  size_t video_index_{0};
};
}
