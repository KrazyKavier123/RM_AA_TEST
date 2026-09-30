#include "Detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rmv {

std::vector<Light> Detector::find_lights(const cv::Mat& bgr, const cv::Mat& mask) const {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //提取亮区轮廓

    std::vector<Light> result;
    for (const auto& contour : contours) {
        if (contour.size() < 4) continue;
        const cv::RotatedRect box = cv::minAreaRect(contour); //计算一个能包住 contour 中所有点、且面积最小的矩形
        cv::Point2f p[4];
        box.points(p);
        cv::Point2f axis = p[1] - p[0];
        if (cv::norm(p[2] - p[1]) > cv::norm(axis)) axis = p[2] - p[1];
        const float len = static_cast<float>(cv::norm(axis));
        const float width = std::min(box.size.width, box.size.height);
        if (len < cfg_.min_light_len || width / len > cfg_.max_light_ratio) continue; //排除过短或过粗的轮廓
        if (axis.y < 0) axis = -axis;  // 统一朝下，便于算倾角
        if (std::abs(std::atan2(axis.x, axis.y)) * 180 / CV_PI > cfg_.max_tilt_deg) continue; //排除倾斜角过大的灯条

        // 用轮廓内的平均颜色判断灯条是红还是蓝。
        const cv::Rect roi = cv::boundingRect(contour) & cv::Rect(0, 0, bgr.cols, bgr.rows);
        cv::Mat roi_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
        std::vector<cv::Point> local;
        for (const auto& point : contour) local.emplace_back(point - roi.tl());
        cv::drawContours(roi_mask, std::vector<std::vector<cv::Point>>{local}, -1, 255, cv::FILLED);
        const cv::Scalar mean = cv::mean(bgr(roi), roi_mask);
        if (std::abs(mean[2] - mean[0]) < cfg_.min_color_diff) continue;
        const TeamColor color = mean[2] > mean[0] ? TeamColor::Red : TeamColor::Blue; //bgr图像取色，红色通道大于蓝色通道则为红灯条，否则为蓝灯条
        if (color != color_) continue;

        result.push_back({box.center - axis * 0.5F, box.center + axis * 0.5F,
                          box.center, len, width});
    }
    std::sort(result.begin(), result.end(),
              [](const Light& a, const Light& b) { return a.center.x < b.center.x; });
    return result;
}

std::vector<ArmorDetection> Detector::match(const std::vector<Light>& lights) const {
    std::vector<ArmorDetection> result;
    for (std::size_t i = 0; i < lights.size(); ++i) {
        for (std::size_t j = i + 1; j < lights.size(); ++j) {
            const Light& left = lights[i];
            const Light& right = lights[j];
            const float len = (left.len + right.len) * 0.5F;
            if (len <= 0 || std::min(left.len, right.len) / std::max(left.len, right.len) <
                                cfg_.min_len_ratio)
                continue; //筛选长宽比
            const float ratio = (right.center.x - left.center.x) / len; //筛选距离
            if (ratio < cfg_.min_pair_ratio || ratio > cfg_.max_pair_ratio ||
                std::abs(right.center.y - left.center.y) / len > cfg_.max_dy_ratio)
                continue;
            // 两灯条之间夹着第三条灯条时不是同一块装甲板。
            bool blocked = false;
            for (std::size_t k = i + 1; k < j; ++k)
                if (std::abs(lights[k].center.y - (left.center.y + right.center.y) * 0.5F) < len)
                    blocked = true;
            if (blocked) continue;

            ArmorDetection det;
            det.corners = {left.top, right.top, right.bottom, left.bottom};
            det.size = ratio >= cfg_.large_pair_ratio ? ArmorSize::Large : ArmorSize::Small;
            result.push_back(det);
        }
    }
    return result;
}

}  // namespace rmv
