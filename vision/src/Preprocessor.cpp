#include "Preprocessor.hpp"

#include <opencv2/imgproc.hpp>

namespace rmv {

cv::Mat Preprocessor::run(const cv::Mat& bgr) const {
    cv::Mat gray, mask;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY); //转为灰度值
    cv::threshold(gray, mask, cfg_.threshold, 255, cv::THRESH_BINARY);
    if (cfg_.open_radius > 0) { //开运算半径大于 0 时做开运算，去除小噪点
        const int k = 2 * cfg_.open_radius + 1;
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k}));
    }
    return mask;
}

}  // namespace rmv
