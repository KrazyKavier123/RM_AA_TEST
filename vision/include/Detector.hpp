#pragma once

#include "AutoAimTypes.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace rmv {

// 灯条。移植自 src/rm_aa_vision/include/rm_aa_vision/types.hpp。
struct Light {
    cv::Point2f top, bottom, center;
    float len{0}, width{0};
};

// 传统视觉检测参数。移植自 src/rm_aa_vision/include/rm_aa_vision/detector.hpp。
struct DetectorConfig {
    float min_light_len{8.0f};     // 灯条最小长度 (px)
    float max_light_ratio{0.45f};  // 灯条 宽/长 上限
    float max_tilt_deg{35.0f};     // 灯条与竖直方向夹角上限 (度)
    float min_pair_ratio{1.5f};    // 灯条中心距/灯条长 下限
    float max_pair_ratio{5.0f};    // 上限
    float large_pair_ratio{3.3f};  // 比值超过该值判为大装甲板
    float min_len_ratio{0.65f};    // 两灯条长度比下限
    float max_dy_ratio{0.6f};      // 两灯条高度差/灯条长 上限
    float min_color_diff{8.0f};    // 红蓝通道差下限
};

class Detector {
public:
    explicit Detector(DetectorConfig config = {}) : cfg_(config) {}

    void set_color(TeamColor color) { color_ = color; }
    std::vector<Light> find_lights(const cv::Mat& bgr, const cv::Mat& mask) const;
    // 由灯条配对生成装甲板；角点顺序为 左上/右上/右下/左下。
    std::vector<ArmorDetection> match(const std::vector<Light>& lights) const;

private:
    DetectorConfig cfg_;
    TeamColor color_{TeamColor::Blue};  // 要检测的敌方颜色
};

}  // namespace rmv
