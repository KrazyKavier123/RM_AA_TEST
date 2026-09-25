#pragma once

#include "AutoAimTypes.hpp"

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

namespace rmv {

// 装甲板数字分类。移植自 src/rm_aa_vision/src/classifier.cpp；
// 模型与标签在 models/ 下（源于 rm_auto_aim，MIT）。
class Classifier {
public:
    Classifier(const std::string& model_path, const std::string& label_path,
               double threshold = 0.5);

    // 填写装甲板编号与置信度；无法映射为 1~5 号时 target_id 置 0。
    void classify(const cv::Mat& bgr, ArmorDetection& det);

private:
    cv::Mat extract(const cv::Mat& bgr, const ArmorDetection& det) const;

    cv::dnn::Net net_;
    std::vector<std::string> labels_;
    double threshold_;
};

}  // namespace rmv
