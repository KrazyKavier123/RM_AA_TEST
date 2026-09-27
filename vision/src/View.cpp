#include "View.hpp"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace rmv {

View::~View() {
    if (opened_) cv::destroyWindow("RMV");
}

bool View::show(const cv::Mat& frame, const std::vector<ArmorDetection>& dets,
                const std::vector<ArmorPose>& poses, const CameraParameters& camera) {
    if (!cfg_.enabled) return true;
    if (!opened_) {
        cv::namedWindow("RMV", cv::WINDOW_NORMAL);
        cv::resizeWindow("RMV", cvRound(frame.cols * cfg_.scale),
                         cvRound(frame.rows * cfg_.scale));
        opened_ = true;
    }

    cv::Mat image = frame.clone();
    for (const auto& det : dets) {
        const cv::Scalar color = det.target_id > 0 ? cv::Scalar(0, 255, 0)
                                                  : cv::Scalar(0, 180, 255);
        for (int i = 0; i < 4; ++i)
            cv::line(image, det.corners[i], det.corners[(i + 1) % 4], color,
                     cfg_.thickness, cv::LINE_AA);
        const std::string label = det.label + cv::format(" %.0f%%", det.confidence * 100)
                                 + (det.target_id == 0 ? " [ignored]" : "");
        const cv::Point text(std::max(0, cvRound(det.corners[0].x)),
                             std::max(20, cvRound(det.corners[0].y) - 8));
        cv::putText(image, label, text, cv::FONT_HERSHEY_SIMPLEX,
                    cfg_.font, color, cfg_.thickness, cv::LINE_AA);
    }
    for (const auto& pose : poses) {
        // 直接使用本帧 PnP 的最优解；X 红、Y 绿、Z 蓝（装甲板 +Z 法线）。
        cv::drawFrameAxes(image, camera.camera_matrix, camera.distortion_coefficients,
                          pose.rvec, pose.position_camera_m, cfg_.axis, cfg_.thickness);
    }
    cv::putText(image, "PnP: X red / Y green / Z blue | Q / Esc: quit",
                {12, 25}, cv::FONT_HERSHEY_SIMPLEX, cfg_.font,
                {255, 255, 255}, cfg_.thickness, cv::LINE_AA);
    cv::imshow("RMV", image);
    const int key = cv::waitKey(1) & 0xff;
    return key != 27 && key != 'q' && key != 'Q' &&
           cv::getWindowProperty("RMV", cv::WND_PROP_VISIBLE) >= 1;
}

}  // namespace rmv
