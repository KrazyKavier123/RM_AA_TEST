#include "Classifier.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace rmv {

Classifier::Classifier(const std::string& model_path, const std::string& label_path,
                       double threshold)
    : net_(cv::dnn::readNetFromONNX(model_path)), threshold_(threshold) {
    std::ifstream input(label_path);
    if (!input) throw std::runtime_error("无法读取标签文件: " + label_path);
    for (std::string line; std::getline(input, line);) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) labels_.push_back(line);
    }
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

cv::Mat Classifier::extract(const cv::Mat& bgr, const ArmorDetection& det) const {
    // ROI 尺寸与归一化沿用 rm_auto_aim：小/大板宽度不同，数字区域固定 20x28。
    const int width = det.size == ArmorSize::Small ? 32 : 54;
    const std::vector<cv::Point2f> src{det.corners[3], det.corners[0],
                                       det.corners[1], det.corners[2]};
    const std::vector<cv::Point2f> dst{{0, 19}, {0, 7},
                                       {static_cast<float>(width - 1), 7},
                                       {static_cast<float>(width - 1), 19}};
    cv::Mat warped, gray, binary;
    cv::warpPerspective(bgr, warped, cv::getPerspectiveTransform(src, dst), {width, 28});
    cv::cvtColor(warped(cv::Rect((width - 20) / 2, 0, 20, 28)), gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    return binary;
}

void Classifier::classify(const cv::Mat& bgr, ArmorDetection& det) {
    net_.setInput(cv::dnn::blobFromImage(extract(bgr, det), 1.0 / 255.0));
    const cv::Mat logits = net_.forward().reshape(1, 1);
    if (logits.total() != labels_.size())
        throw std::runtime_error("分类网络输出维度与标签数不一致");

    double maximum = 0.0;
    cv::minMaxLoc(logits, nullptr, &maximum);
    cv::Mat prob;
    cv::exp(logits - maximum, prob);
    prob /= cv::sum(prob)[0];
    cv::Point index;
    double confidence = 0.0;
    cv::minMaxLoc(prob, nullptr, &confidence, nullptr, &index);

    const std::string& label = labels_.at(index.x);
    det.confidence = static_cast<float>(confidence);
    det.target_id = 0;
    if (confidence >= threshold_ && label.size() == 1 && label[0] >= '1' && label[0] <= '5')
        det.target_id = label[0] - '0';  // 只接受 1~5 号装甲板
}

}  // namespace rmv
