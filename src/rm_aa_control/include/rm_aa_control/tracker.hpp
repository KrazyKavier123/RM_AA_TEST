#pragma once
#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace rm_aa_control {
constexpr double pi = 3.14159265358979323846;
double wrap(double angle);
struct Observation {
  Eigen::Vector3d position;
  double yaw;  // Heading of the plate's inward normal in the fixed frame.
};
struct TrackerConfig {
  int plate_count{4};
  double radius{0.25}, min_radius{0.05}, max_radius{0.8};
  double acceleration_variance{4.0}, angular_acceleration_variance{16.0};
  double position_variance{0.0025}, yaw_variance{0.04};
  double association_distance{0.5}, association_yaw{1.0}, nis_gate{18.47};
  double timeout{0.3}, switch_margin{0.05};
  bool alternating_geometry{false};
  double max_height_difference{0.3};
};
class Tracker {
public:
  // [cx, vx, cy, vy, z_even, vz, heading, heading_rate, r_even, r_odd, dz_odd]
  using State = Eigen::Matrix<double, 11, 1>;
  using Covariance = Eigen::Matrix<double, 11, 11>;
  explicit Tracker(TrackerConfig config = {});
  bool update(const std::vector<Observation> & observations, double stamp,
              const Eigen::Vector3d & camera_position);
  void reset();
  bool valid(double stamp) const;
  State state_at(double stamp) const;
  Observation plate_at(int index, double stamp) const;
  // Distance in metres, with hysteresis; index is stable within this track.
  int select_nearest(double stamp, const Eigen::Vector3d & camera_position);
  const Covariance & covariance() const { return covariance_; }
  Covariance covariance_at(double stamp) const;
  double last_seen() const { return last_seen_; }
private:
  Observation measurement(const State & state, int index) const;
  Eigen::Matrix<double, 4, 11> jacobian(int index) const;
  void predict(double stamp);
  bool correct(const Observation & observation, int index);
  TrackerConfig config_;
  State state_{State::Zero()};
  Covariance covariance_{Covariance::Identity()};
  bool initialized_{false};
  double stamp_{0}, last_seen_{0};
  int selected_{-1};
};
}
