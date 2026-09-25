#pragma once

#include <opencv2/core.hpp>

namespace rmv {

// 灰度二值化。移植自 src/rm_aa_vision/src/preprocessor.cpp。
struct PreprocessConfig {
    int threshold{100};   // 亮度阈值，0~255
    int open_radius{0};   // 开运算半径，0 表示不做
};

class Preprocessor {
public:
    explicit Preprocessor(PreprocessConfig config = {}) : cfg_(config) {}
    cv::Mat run(const cv::Mat& bgr) const;  // 返回 CV_8UC1 掩膜

private:
    PreprocessConfig cfg_;
};

}  // namespace rmv
