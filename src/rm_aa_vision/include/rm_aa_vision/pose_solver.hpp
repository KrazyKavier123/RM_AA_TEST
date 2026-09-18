#pragma once
#include "rm_aa_vision/types.hpp"
namespace rm_aa {
class PoseSolver {
public:
  explicit PoseSolver(Dimensions dimensions = {}, double max_error_px = 3.0);
  std::vector<cv::Point3d> object_points(PlateSize size) const;
  std::optional<PoseObservation> solve(const Plate & plate, const Calibration & camera) const;
private:
  Dimensions dimensions_;
  double max_error_;
};
}
