#include "rm_aa_hardware/protocol.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace rm_aa_hardware {
namespace {
constexpr double tau = 6.28318530717958647692;
bool wire_float(double value) { return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max(); }
float finite_or_zero(float value) { return std::isfinite(value) ? value : 0.0F; }
}
bool wire_platform_supported() noexcept {
  const unsigned short endian = 1;
  return sizeof(float) == 4 && std::numeric_limits<float>::is_iec559 &&
    *reinterpret_cast<const unsigned char *>(&endian) == 1;
}
bool valid_feedback(const MessData_AutoAim & f) noexcept {
  return f.head == kFrameHead && f.tail == kFrameTail && std::isfinite(f.yaw) &&
    std::isfinite(f.pitch) && std::isfinite(f.roll) && std::isfinite(f.bias);
}
std::vector<Translator> FrameParser::append(const char * data, std::size_t size) {
  std::vector<Translator> result;
  // Process incrementally so noise cannot grow the retained buffer without bound.
  for (std::size_t i = 0; i < size; ++i) {
    pending_.push_back(data[i]);
    while (!pending_.empty() && static_cast<unsigned char>(pending_.front()) != kFrameHead)
      pending_.erase(pending_.begin());
    if (pending_.size() < kCommunicationFrameSize) continue;
    Translator frame{};
    std::memcpy(frame.data, pending_.data(), kCommunicationFrameSize);
    if (valid_feedback(frame.message)) {
      result.push_back(frame);
      pending_.clear();
    } else {
      pending_.erase(pending_.begin());
    }
  }
  return result;
}
bool valid_command(const Command & c) noexcept {
  return c.target_id > 0 && c.target_id <= 255 && (c.tracking_state == 1 || c.tracking_state == 2) &&
    wire_float(c.yaw) && wire_float(c.pitch) && wire_float(c.yaw_velocity) && wire_float(c.pitch_velocity) &&
    wire_float(c.distance_m) && c.distance_m >= 0 && wire_float(c.center_x_m * 1000) &&
    wire_float(c.center_y_m * 1000) && wire_float(c.latency_ms) && c.latency_ms >= 0;
}
Translator output_frame(const MessData_AutoAim & feedback, const Command * command,
                        bool feedback_fresh, bool enable_fire) noexcept {
  Translator result{};
  auto & m = result.message;
  m.head = kFrameHead; m.tail = kFrameTail;
  m.yaw = finite_or_zero(feedback.yaw); m.pitch = finite_or_zero(feedback.pitch);
  m.roll = finite_or_zero(feedback.roll); m.bias = finite_or_zero(feedback.bias);
  m.status = feedback.status; m.is_far = feedback.is_far;
  m.target_yaw = m.yaw; m.target_pitch = m.pitch;
  if (!feedback_fresh || !valid_feedback(feedback) || (m.status != 0 && m.status != 5) ||
      !command || !valid_command(*command)) return result;
  const auto & c = *command;
  m.armor_flag = static_cast<std::uint8_t>(c.target_id);
  m.crc = static_cast<std::uint16_t>(c.tracking_state);
  m.target_yaw = static_cast<float>(c.yaw); m.target_pitch = static_cast<float>(c.pitch);
  m.yaw_vel = static_cast<float>(c.yaw_velocity); m.pitch_vel = static_cast<float>(c.pitch_velocity);
  m.distance = static_cast<float>(c.distance_m);
  m.coo_x = static_cast<float>(c.center_x_m * 1000); m.coo_y = static_cast<float>(c.center_y_m * 1000);
  m.latency = static_cast<float>(c.latency_ms);
  m.fire_allowance = enable_fire && c.fire && c.tracking_state == 1 ? 1 : 0;
  return result;
}
AttitudeSample AttitudeEstimator::update(const MessData_AutoAim & f, double t, double max_gap) {
  AttitudeSample result;
  result.position = {f.yaw, f.pitch, f.roll};
  const double dt = t - time_;
  if (initialized_ && dt >= 1e-4 && dt <= max_gap) {
    for (std::size_t i = 0; i < 3; ++i)
      result.velocity[i] = std::remainder(result.position[i] - previous_[i], tau) / dt;
    result.velocity_valid = true;
  }
  previous_ = result.position; time_ = t; initialized_ = true;
  return result;
}
std::array<double, 4> attitude_quaternion(double yaw, double pitch, double roll) {
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  const double cp = std::cos(-pitch / 2), sp = std::sin(-pitch / 2);
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  return {sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy};
}
bool valid_trajectory(const std::vector<TrajectoryPoint> & points) noexcept {
  if (points.empty()) return false;
  double previous = -1;
  for (const auto & p : points) {
    if (!std::isfinite(p.time) || p.time < 0 || p.time <= previous) return false;
    for (std::size_t i = 0; i < 2; ++i)
      if (!wire_float(p.position[i]) || !wire_float(p.velocity[i])) return false;
    previous = p.time;
  }
  return true;
}
bool sample_trajectory(const std::vector<TrajectoryPoint> & points, double elapsed,
                       const std::array<double, 2> & position,
                       const std::array<double, 2> & velocity, Command & output) {
  if (!valid_trajectory(points) || !std::isfinite(elapsed) || elapsed < 0 || elapsed > points.back().time)
    return false;
  const auto upper = std::lower_bound(points.begin(), points.end(), elapsed,
    [](const auto & p, double t) { return p.time < t; });
  if (upper == points.end()) return false;
  const TrajectoryPoint left = upper == points.begin() ? TrajectoryPoint{0, position, velocity} : *(upper - 1);
  const double duration = upper->time - left.time;
  const double ratio = duration > 0 ? (elapsed - left.time) / duration : 1;
  // Yaw trajectory is unwrapped by the planner. Do not wrap it a second time.
  output.yaw = left.position[0] + ratio * (upper->position[0] - left.position[0]);
  output.pitch = left.position[1] + ratio * (upper->position[1] - left.position[1]);
  output.yaw_velocity = left.velocity[0] + ratio * (upper->velocity[0] - left.velocity[0]);
  output.pitch_velocity = left.velocity[1] + ratio * (upper->velocity[1] - left.velocity[1]);
  return true;
}
}
