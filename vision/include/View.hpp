#pragma once

#include "AutoAimTypes.hpp"

namespace rmv {

// 显示参数集中在这里；坐标轴长度用米，画面缩放不影响检测和 PnP。
struct ViewConfig {
    bool enabled{true};
    double scale{0.75};
    float axis{0.08f};
    int thickness{2};
    double font{0.6};
};

class View {
public:
    explicit View(ViewConfig config = {}) : cfg_(config) {}
    ~View();
    // 返回 false 表示按下 Q / Esc 或关闭窗口。
    bool show(const cv::Mat& frame, const std::vector<ArmorDetection>& dets,
              const std::vector<ArmorPose>& poses, const CameraParameters& camera);

private:
    ViewConfig cfg_;
    bool opened_{false};
};

}  // namespace rmv
