#pragma once

#include "AutoAimTypes.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace rmv {

// PnP 位姿解算。移植自 src/rm_aa_vision/src/pose_solver.cpp。
class PoseSolver {
public:
    PoseSolver(double small_w_m = 0.135, double large_w_m = 0.230, double h_m = 0.057,
               double max_err_px = 3.0, double normal_sign = 1.0);

    // 输出云台坐标系下的位置、装甲板朝向和重投影误差。
    std::vector<ArmorPose> solve(const std::vector<ArmorDetection>& dets,
                                 const CameraParameters& camera) const;

private:
    std::vector<cv::Point3d> object_points(ArmorSize size) const;

    double small_w_, large_w_, h_, max_err_, normal_sign_;
};

}  // namespace rmv
