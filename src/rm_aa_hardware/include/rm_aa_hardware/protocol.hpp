#pragma once
#include "Communication.hpp"
#include <array>
#include <cstddef>
#include <vector>

namespace rm_aa_hardware {
// Communication.hpp is the SAME source file used by training; crc is a state, not a checksum.
bool wire_platform_supported() noexcept;
bool valid_feedback(const MessData_AutoAim & frame) noexcept;
class FrameParser {
public:
  std::vector<Translator> append(const char * data, std::size_t size);
  void reset() { pending_.clear(); }
private:
  std::vector<char> pending_;
};
struct Command {
  double yaw{}, pitch{}, yaw_velocity{}, pitch_velocity{};
  double distance_m{}, center_x_m{}, center_y_m{}, latency_ms{};
  unsigned target_id{}, tracking_state{};
  bool fire{};
};
bool valid_command(const Command & command) noexcept;
Translator output_frame(const MessData_AutoAim & feedback, const Command * command,
                        bool feedback_fresh, bool enable_fire) noexcept;

struct AttitudeSample {
  std::array<double, 3> position{};  // yaw, pitch (positive up), roll
  std::array<double, 3> velocity{};
  bool velocity_valid{};
};
class AttitudeEstimator {
public:
  AttitudeSample update(const MessData_AutoAim & frame, double steady_seconds, double max_gap);
  void reset() { initialized_ = false; }
private:
  bool initialized_{};
  double time_{};
  std::array<double, 3> previous_{};
};
// Quaternion [x,y,z,w] for Rz(yaw) * Ry(-pitch) * Rx(roll).
std::array<double, 4> attitude_quaternion(double yaw, double pitch, double roll);
struct TrajectoryPoint {
  double time{};
  std::array<double, 2> position{}, velocity{};
};
bool valid_trajectory(const std::vector<TrajectoryPoint> & points) noexcept;
bool sample_trajectory(const std::vector<TrajectoryPoint> & points, double elapsed,
                       const std::array<double, 2> & initial_position,
                       const std::array<double, 2> & initial_velocity, Command & output);
}
