#include "rm_aa_vision/source.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <opencv2/imgproc.hpp>
namespace rm_aa {
cv::Mat synthetic_frame(int width, int height, double time_s) {
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(12, 16, 22));
  const auto draw = [&](double x, double y, double ratio, cv::Scalar color, const char * label) {
    const int length = std::max(12, height / 14), spacing = static_cast<int>(length * ratio);
    const int cx = static_cast<int>(x * width), cy = static_cast<int>(y * height);
    for (int sign : {-1, 1}) {
      const int lx = cx + sign * spacing / 2;
      cv::rectangle(image, cv::Rect(lx - 3, cy - length / 2, 7, length + 1), color, cv::FILLED);
    }
    cv::putText(image, label, {cx - 13, cy + 15}, cv::FONT_HERSHEY_SIMPLEX, 1.3, {240,240,240}, 2);
  };
  draw(0.28 + 0.07 * std::sin(time_s), 0.38, 2.45, {255,180,180}, "3");
  draw(0.68 + 0.04 * std::sin(time_s * 0.7), 0.68, 4.09, {255,180,180}, "1");
  draw(0.65, 0.22, 2.45, {100,100,255}, "4");
  return image;
}
Source::Source(SourceConfig config) : config_(std::move(config)) {
  if (config_.width < 320 || config_.height < 240 || !std::isfinite(config_.fps) || config_.fps <= 0 || config_.fps > 500)
    throw std::invalid_argument("Invalid capture resolution/fps");
  if (config_.mode == "synthetic") return;
  if (config_.mode == "video") capture_.open(config_.video_path);
  else if (config_.mode == "camera") {
    const bool numeric = !config_.device.empty() && std::all_of(config_.device.begin(), config_.device.end(),
      [](unsigned char c) { return std::isdigit(c); });
#ifdef __linux__
    const int api = cv::CAP_V4L2;
#else
    const int api = cv::CAP_ANY;
#endif
    if (numeric) capture_.open(std::stoi(config_.device), api); else capture_.open(config_.device, api);
    if (capture_.isOpened()) {
      capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
      capture_.set(cv::CAP_PROP_FPS, config_.fps);
      capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    }
  } else throw std::invalid_argument("Source mode must be synthetic, camera or video");
  if (!capture_.isOpened()) throw std::runtime_error("Cannot open capture source");
}
double Source::nominal_fps() const {
  const double value = config_.mode == "video" ? capture_.get(cv::CAP_PROP_FPS) : config_.fps;
  return std::isfinite(value) && value > 0 ? value : config_.fps;
}
bool Source::read(cv::Mat & image) {
  if (config_.mode == "synthetic") {
    image = synthetic_frame(config_.width, config_.height, index_++ / config_.fps); return true;
  }
  if (capture_.read(image) && !image.empty()) return true;
  if (config_.mode == "video" && config_.loop_video) {
    capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
    return capture_.read(image) && !image.empty();
  }
  return false;
}
}
