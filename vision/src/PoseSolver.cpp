#include "PoseSolver.hpp"

#include <cmath>

#include <opencv2/calib3d.hpp>

namespace rmv {

PoseSolver::PoseSolver(double small_w_m, double large_w_m, double h_m, double max_err_px,
                       double normal_sign)
    : small_w_(small_w_m), large_w_(large_w_m), h_(h_m), max_err_(max_err_px),
      normal_sign_(normal_sign) {}

std::vector<cv::Point3d> PoseSolver::object_points(ArmorSize size) const {
    const double w = (size == ArmorSize::Small ? small_w_ : large_w_) / 2.0;
    const double h = h_ / 2.0;
    // 物点平面 z=0，顺序与角点 左上/右上/右下/左下 对应。
    return {{-w, -h, 0}, {w, -h, 0}, {w, h, 0}, {-w, h, 0}};
}

std::vector<ArmorPose> PoseSolver::solve(const std::vector<ArmorDetection>& dets,
                                         const CameraParameters& camera) const {
    std::vector<ArmorPose> result;
    const cv::Mat& K = camera.camera_matrix;
    const cv::Mat& D = camera.distortion_coefficients;
    const cv::Mat& R_cg = camera.rotation_camera_to_gimbal;
    const cv::Mat& t_cg = camera.translation_camera_to_gimbal;

    for (const ArmorDetection& det : dets) {
        const std::vector<cv::Point3d> obj = object_points(det.size);
        const std::vector<cv::Point2d> img(det.corners.begin(), det.corners.end());
        std::vector<cv::Mat> rvecs, tvecs;
        try {
            cv::solvePnPGeneric(obj, img, K, D, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
        } catch (const cv::Exception&) {
            continue;
        }

        ArmorPose best;
        double best_error = max_err_;
        for (std::size_t i = 0; i < rvecs.size(); ++i) {
            const cv::Vec3d r(rvecs[i].at<double>(0), rvecs[i].at<double>(1),
                              rvecs[i].at<double>(2));
            const cv::Vec3d t(tvecs[i].at<double>(0), tvecs[i].at<double>(1),
                              tvecs[i].at<double>(2));
            cv::Matx33d R;
            cv::Rodrigues(r, R);
            bool in_front = true;
            for (const cv::Point3d& p : obj)
                if ((R * cv::Vec3d(p.x, p.y, p.z) + t)[2] <= 0) in_front = false;
            if (!in_front) continue;

            std::vector<cv::Point2d> projected;
            cv::projectPoints(obj, r, t, K, D, projected);
            double squared = 0.0;
            for (std::size_t j = 0; j < obj.size(); ++j) {
                const cv::Point2d delta = projected[j] - img[j];
                squared += delta.dot(delta);
            }
            const double error = std::sqrt(squared / obj.size());
            if (error >= best_error) continue;  // 保留重投影误差最小的解

            // 装甲板法线（物点 +z）只做旋转转到云台系，取其偏航朝向。
            const cv::Vec3d normal_cam = R * cv::Vec3d(0, 0, 1);
            const cv::Mat n_gimbal = R_cg * cv::Mat(normal_cam);

            best_error = error;
            best.detection = det;
            best.position_camera_m = t;
            best.reprojection_error = error;
            best.armor_yaw = normal_sign_ *
                             std::atan2(n_gimbal.at<double>(1), n_gimbal.at<double>(0));
            best.position_gimbal_m = cv::Vec3d(
                R_cg.at<double>(0, 0) * t[0] + R_cg.at<double>(0, 1) * t[1] +
                    R_cg.at<double>(0, 2) * t[2] + t_cg.at<double>(0),
                R_cg.at<double>(1, 0) * t[0] + R_cg.at<double>(1, 1) * t[1] +
                    R_cg.at<double>(1, 2) * t[2] + t_cg.at<double>(1),
                R_cg.at<double>(2, 0) * t[0] + R_cg.at<double>(2, 1) * t[1] +
                    R_cg.at<double>(2, 2) * t[2] + t_cg.at<double>(2));
        }
        if (best_error < max_err_) result.push_back(best);
    }
    return result;
}

}  // namespace rmv
