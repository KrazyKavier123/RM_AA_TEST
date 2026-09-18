#include "rm_aa_vision/pose_solver.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
namespace rm_aa {
PoseSolver::PoseSolver(Dimensions dimensions, double max_error_px)
  : dimensions_(dimensions), max_error_(max_error_px) {
  if (!std::isfinite(dimensions.small_width_m) || !std::isfinite(dimensions.large_width_m) ||
      !std::isfinite(dimensions.height_m) || !std::isfinite(max_error_px) ||
      dimensions.small_width_m <= 0 || dimensions.large_width_m <= dimensions.small_width_m ||
      dimensions.height_m <= 0 || max_error_px <= 0) throw std::invalid_argument("Invalid plate dimensions/error limit");
}
std::vector<cv::Point3d> PoseSolver::object_points(PlateSize size) const {
  const double w = (size == PlateSize::Small ? dimensions_.small_width_m : dimensions_.large_width_m) / 2;
  const double h = dimensions_.height_m / 2;
  return {{-w, -h, 0}, {w, -h, 0}, {w, h, 0}, {-w, h, 0}};
}
std::optional<PoseObservation> PoseSolver::solve(const Plate & plate, const Calibration & camera) const {
  if (!camera.calibrated || camera.matrix(0, 0) <= 0 || camera.matrix(1, 1) <= 0 ||
      !cv::checkRange(cv::Mat(camera.matrix))) return std::nullopt;
  const auto points = object_points(plate.size);
  std::vector<cv::Point2d> image_points;
  for (const auto & p : plate.corners) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return std::nullopt;
    image_points.emplace_back(p.x, p.y);
  }
  std::vector<cv::Mat> rotations, translations;
  try {
    cv::solvePnPGeneric(points, image_points, camera.matrix, camera.distortion,
      rotations, translations, false, cv::SOLVEPNP_IPPE);
  } catch (const cv::Exception &) { return std::nullopt; }
  std::optional<PoseObservation> best;
  for (size_t i = 0; i < rotations.size(); ++i) {
    if (!cv::checkRange(rotations[i]) || !cv::checkRange(translations[i])) continue;
    const cv::Vec3d r(rotations[i].at<double>(0), rotations[i].at<double>(1), rotations[i].at<double>(2));
    const cv::Vec3d t(translations[i].at<double>(0), translations[i].at<double>(1), translations[i].at<double>(2));
    cv::Matx33d rotation; cv::Rodrigues(r, rotation);
    bool in_front = true;
    for (const auto & p : points) if ((rotation * cv::Vec3d(p.x, p.y, p.z) + t)[2] <= 0) in_front = false;
    if (!in_front) continue;
    std::vector<cv::Point2d> projected;
    cv::projectPoints(points, r, t, camera.matrix, camera.distortion, projected);
    double squared_error = 0;
    for (size_t j = 0; j < points.size(); ++j) {
      const auto delta = projected[j] - image_points[j];
      squared_error += delta.dot(delta);
    }
    const double error = std::sqrt(squared_error / points.size());
    if (!std::isfinite(error) || error > max_error_ || (best && error >= best->reprojection_rmse_px)) continue;
    // Camera optical coordinates: x right, y down, z forward. Observation bearings only.
    best = PoseObservation{r, t, error, std::atan2(t[0], t[2]), -std::atan2(t[1], std::hypot(t[0], t[2]))};
  }
  return best;
}
}
