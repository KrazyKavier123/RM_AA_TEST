#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace rmv {

// 取流来源。移植自 src/rm_aa_vision/src/source.cpp，
// 只保留真实相机（UVC/V4L2 或海康 MVS SDK）与读视频两条链路，不含合成数据。
struct SourceConfig {
    std::string mode{"video"};      // "camera"：真实相机；"video"：视频文件或目录
    std::string backend{"uvc"};     // camera 模式："uvc"（OpenCV）或 "hikrobot"（MVS SDK）
    std::string device{"0"};        // uvc：设备号或 /dev/videoX；hikrobot：枚举序号
    std::string serial;             // 非空时按海康序列号选相机，优先于 device
    std::string path;               // video 模式：文件或目录
    int width{1280};
    int height{720};
    double fps{60.0};
    double exposure_us{20000.0};    // 海康相机曝光时间
    double gain_db{16.0};           // 海康相机增益
    int grab_timeout_ms{1000};      // 等待一帧的最长时间
    bool loop{false};               // 视频播放结束后从头循环
};

class Source {
public:
    explicit Source(SourceConfig config);
    ~Source();

    // 取一帧 BGR 图像。成功返回 true；输入结束、超时或打开失败返回 false。
    bool read(cv::Mat& image);
    const SourceConfig& config() const { return config_; }

private:
    struct HikCamera;  // 海康 MVS SDK 取流状态，仅在 ENABLE_HIKROBOT 编译时构造
    void use_video_fps();

    SourceConfig config_;
    cv::VideoCapture capture_;
    std::unique_ptr<HikCamera> hik_;
    std::vector<std::string> files_;
    std::size_t index_{0};
    std::chrono::steady_clock::time_point next_{};  // 视频回放的下一帧时刻
};

}  // namespace rmv
