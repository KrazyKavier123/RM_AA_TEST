#include "rm_aa_control/tracker.hpp"
#include "rm_aa_control/mpc.hpp"
#include "rm_aa_control/ballistic.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace rm_aa_control {
class ControlNode final : public rclcpp::Node {
public:
  ControlNode() : Node("camera_control"), buffer_(get_clock()), listener_(buffer_) {
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    base_frame_ = declare_parameter<std::string>("gimbal_base_frame", "gimbal_base");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    const auto camera_to_pitch = declare_parameter<std::vector<double>>(
      "camera_to_pitch_optical_m", std::vector<double>{0.0, 0.0, 0.0});
    if (camera_to_pitch.size() != 3 ||
        !std::all_of(camera_to_pitch.begin(), camera_to_pitch.end(),
                     [](double value) { return std::isfinite(value); }))
      throw std::invalid_argument("camera_to_pitch_optical_m must contain three finite metres");
    // Negate camera -> P, then map optical right/down/forward to forward/left/up.
    pitch_to_camera_ = {-camera_to_pitch[2], camera_to_pitch[0], camera_to_pitch[1]};
    camera_offset_tolerance_ = positive("camera_offset_tolerance_m", 0.005);
    const auto pitch_to_barrel = declare_parameter<std::vector<double>>("pitch_to_barrel_m", std::vector<double>{0.0741, 0.0, 0.0});
    if (pitch_to_barrel.size() != 3 ||
        !std::all_of(pitch_to_barrel.begin(), pitch_to_barrel.end(),
                     [](double value) { return std::isfinite(value); }))
      throw std::invalid_argument("pitch_to_barrel_m must contain three finite metres");
    pitch_to_barrel_ = Eigen::Vector3d(
      pitch_to_barrel[0], pitch_to_barrel[1], pitch_to_barrel[2]);
    bullet_speed_ = positive("ballistics.bullet_speed_mps", 22.0);
    gravity_ = positive("ballistics.gravity_mps2", 9.7833);
    yaw_joint_ = declare_parameter<std::string>("yaw_joint", "gimbal_yaw_joint");
    pitch_joint_ = declare_parameter<std::string>("pitch_joint", "gimbal_pitch_joint");
    roll_joint_ = declare_parameter<std::string>("roll_joint", "gimbal_roll_joint");
    timeout_ = positive("feedback_timeout_s", 0.15);
    age_limit_ = positive("observation_max_age_s", 0.2);
    normal_sign_ = declare_parameter<double>("plate_normal_sign", 1.0);
    if (normal_sign_ != 1.0 && normal_sign_ != -1.0) throw std::invalid_argument("plate_normal_sign must be +/-1");
    if (fixed_frame_.empty() || base_frame_.empty() || camera_frame_.empty() ||
        yaw_joint_.empty() || pitch_joint_.empty() || yaw_joint_ == pitch_joint_)
      throw std::invalid_argument("Frames/joint names must be nonempty and joint names distinct");
    TrackerConfig t;
    t.alternating_geometry = declare_parameter<bool>("tracker.alternating_geometry", true);
    t.max_height_difference = positive("tracker.max_height_difference_m", 0.3);
    t.plate_count = declare_parameter<int>("tracker.plate_count", t.plate_count);
    t.radius = positive("tracker.radius_m", t.radius);
    t.min_radius = positive("tracker.min_radius_m", t.min_radius);
    t.max_radius = positive("tracker.max_radius_m", t.max_radius);
    t.acceleration_variance = positive("tracker.acceleration_variance", t.acceleration_variance);
    t.angular_acceleration_variance = positive("tracker.angular_acceleration_variance", t.angular_acceleration_variance);
    t.position_variance = positive("tracker.position_variance", t.position_variance);
    t.yaw_variance = positive("tracker.yaw_variance", t.yaw_variance);
    t.association_distance = positive("tracker.association_distance_m", t.association_distance);
    t.association_yaw = positive("tracker.association_yaw_rad", t.association_yaw);
    t.nis_gate = positive("tracker.nis_gate", t.nis_gate);
    t.timeout = positive("tracker.timeout_s", t.timeout);
    t.switch_margin = declare_parameter<double>("tracker.switch_margin_m", t.switch_margin);
    tracker_ = std::make_unique<Tracker>(t);
    mpc_config_.horizon = declare_parameter<int>("mpc.horizon", mpc_config_.horizon);
    mpc_config_.iterations = declare_parameter<int>("mpc.iterations", mpc_config_.iterations);
    mpc_config_.dt = positive("mpc.dt_s", mpc_config_.dt);
    mpc_config_.angle_weight = positive("mpc.angle_weight", mpc_config_.angle_weight);
    mpc_config_.velocity_weight = positive("mpc.velocity_weight", mpc_config_.velocity_weight);
    mpc_config_.effort_weight = positive("mpc.effort_weight", mpc_config_.effort_weight);
    mpc_config_.tolerance = positive("mpc.tolerance", mpc_config_.tolerance);
    mpc_config_.max_acceleration = positive("mpc.yaw_max_acceleration", 6.0);
    yaw_mpc_ = std::make_unique<AxisMpc>(mpc_config_);
    mpc_config_.max_acceleration = positive("mpc.pitch_max_acceleration", 4.0);
    pitch_mpc_ = std::make_unique<AxisMpc>(mpc_config_);
    auto qos = rclcpp::SensorDataQoS().keep_last(5);
    observations_ = create_subscription<geometry_msgs::msg::PoseArray>(
      declare_parameter<std::string>("observations_topic", "/observations"), qos,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) { observe(*msg); });
    feedback_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("feedback_topic", "/gimbal/joint_states"), qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { feedback(*msg); });
    trajectory_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("gimbal/trajectory", 1);
    state_ = create_publisher<nav_msgs::msg::Odometry>("target/state", 1);
    plate_ = create_publisher<geometry_msgs::msg::PointStamped>("target/selected_plate", 1);
    status_ = create_publisher<std_msgs::msg::String>("control/status", 1);
    parameters_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & ps) {
      rcl_interfaces::msg::SetParametersResult r; r.successful = true;
      for (const auto & p : ps) if (p.get_name() != "use_sim_time") {
        r.successful = false; r.reason = "Control parameters are startup-only; edit YAML and restart"; break;
      }
      return r;
    });
    timer_ = create_wall_timer(std::chrono::duration<double>(mpc_config_.dt), [this] { tick(); });
  }
private:
  double positive(const std::string & name, double value) {
    value = declare_parameter<double>(name, value);
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name + " must be finite and positive");
    return value;
  }
  Eigen::Isometry3d transform(const std::string & to, const std::string & from, const rclcpp::Time & time) {
    if (from.empty()) throw std::runtime_error("Empty observation frame");
    if (to == from) return Eigen::Isometry3d::Identity();
    const auto t = buffer_.lookupTransform(to, from, time).transform;
    Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6) throw std::runtime_error("Invalid TF quaternion");
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = q.normalized().toRotationMatrix();
    result.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
    if (!result.matrix().allFinite()) throw std::runtime_error("Nonfinite TF");
    return result;
  }
  void observe(const geometry_msgs::msg::PoseArray & msg) {
    const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || age < 0 || age > age_limit_) return;
    try {
      const auto world_from_observation = transform(fixed_frame_, msg.header.frame_id, stamp);
      const auto world_from_camera = transform(fixed_frame_, camera_frame_, stamp);
      std::vector<Observation> observations;
      for (const auto & p : msg.poses) {
        const Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
        Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        if (!position.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-6) continue;
        const Eigen::Vector3d inward = normal_sign_ * world_from_observation.linear() *
          (q.normalized() * Eigen::Vector3d::UnitZ());
        if (inward.head<2>().norm() < 0.2) continue;
        observations.push_back({world_from_observation * position, std::atan2(inward.y(), inward.x())});
      }
      tracker_->update(observations, stamp.seconds(), world_from_camera.translation());
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Observation TF unavailable: %s", e.what());
    }
  }
  void feedback(const sensor_msgs::msg::JointState & msg) {
    const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || age < 0 || age > timeout_ || stamp.seconds() <= feedback_stamp_) return;
    Eigen::Vector2d axes[2];
    const std::string names[]{yaw_joint_, pitch_joint_};
    for (int i = 0; i < 2; ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), names[i]);
      if (it == msg.name.end()) return;
      const size_t index = static_cast<size_t>(it - msg.name.begin());
      if (index >= msg.position.size() || index >= msg.velocity.size()) return;
      axes[i] = {msg.position[index], msg.velocity[index]};
      if (!axes[i].allFinite()) return;
    }
    double roll = 0.0;  // Older two-joint feedback (including mock) has no roll axis.
    const auto roll_it = std::find(msg.name.begin(), msg.name.end(), roll_joint_);
    if (roll_it != msg.name.end()) {
      const auto index = static_cast<size_t>(roll_it - msg.name.begin());
      if (index >= msg.position.size() || !std::isfinite(msg.position[index])) return;
      roll = msg.position[index];
    }
    yaw_ = axes[0]; pitch_ = axes[1]; roll_ = roll; feedback_stamp_ = stamp.seconds();
    feedback_stamp_ns_ = stamp.nanoseconds();
  }
  void publish_status(const std::string & value) {
    std_msgs::msg::String msg; msg.data = value; status_->publish(msg);
  }
  void tick() {
    const auto time = now(); const double t = time.seconds();
    if (last_tick_ >= 0 && t < last_tick_) {
      tracker_->reset(); feedback_stamp_ = -1; last_tick_ = t;
      publish_status("clock_reset"); return;
    }
    if (t == last_tick_) return;
    last_tick_ = t;
    if (!tracker_->valid(t)) { publish_status("waiting_for_target"); return; }
    if (feedback_stamp_ < 0 || t < feedback_stamp_ || t - feedback_stamp_ > timeout_) {
      publish_status("waiting_for_feedback"); return;
    }
    try {
      // Use feedback time: dynamic TF often cannot extrapolate to the timer's current time.
      const rclcpp::Time frame_time(feedback_stamp_ns_, get_clock()->get_clock_type());
      const auto base_from_world = transform(base_frame_, fixed_frame_, frame_time);
      
      const auto world_from_camera = transform(fixed_frame_, camera_frame_, frame_time);
      const Eigen::Vector3d camera_in_base =
        (base_from_world * world_from_camera).translation();
      const Eigen::Matrix3d feedback_rotation =
        (Eigen::AngleAxisd(yaw_[0], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-pitch_[0], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())).toRotationMatrix();
      const double camera_offset_error =
        (camera_in_base - feedback_rotation * pitch_to_camera_).norm();
      if (camera_offset_error > camera_offset_tolerance_) {
        publish_status("camera_tf_mismatch");
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Camera TF offset differs from calibration by %.3f m",
                             camera_offset_error);
        return;
      }
      const int index = tracker_->select_nearest(t, world_from_camera.translation());
      if (index < 0) { publish_status("waiting_for_target"); return; }
      Eigen::Vector2d yaw = yaw_, pitch = pitch_;
      yaw[0] += yaw[1] * (t - feedback_stamp_); pitch[0] += pitch[1] * (t - feedback_stamp_);
      std::vector<Eigen::Vector2d> yaw_ref, pitch_ref;
      // Solve the camera ray with its offset rotating about the pitch axis.
            // Solve a firing solution for each future command time. The target is
      // propagated by the bullet flight time before the final aim is computed.
      auto angles = [&](double stamp) -> Eigen::Vector2d {
        auto direct = [&](double fire_stamp) {
          const Eigen::Vector3d target =
            base_from_world * tracker_->plate_at(index, fire_stamp).position;
          return ballistic_pointing_angles(
            target, pitch_to_barrel_, bullet_speed_, gravity_);
        };
        auto solution = direct(stamp);
        for (int iteration = 0; iteration < 3; ++iteration) {
          const auto predicted = direct(stamp + solution.flight_time);
          const bool converged =
            std::abs(predicted.flight_time - solution.flight_time) < 1e-6;
          solution = predicted;
          if (converged) break;
        }
        return solution.angles;
      };
      auto previous = angles(t);
      previous[0] = yaw[0] + wrap(previous[0] - yaw[0]);
      for (int k = 0; k < mpc_config_.horizon; ++k) {
        auto current = angles(t + (k + 1) * mpc_config_.dt);
        current[0] = previous[0] + wrap(current[0] - previous[0]);
        const Eigen::Vector2d velocity = (current - previous) / mpc_config_.dt;
        yaw_ref.emplace_back(current[0], velocity[0]); pitch_ref.emplace_back(current[1], velocity[1]);
        previous = current;
      }
      const auto y = yaw_mpc_->solve(yaw, yaw_ref), p = pitch_mpc_->solve(pitch, pitch_ref);
      if (!y.converged || !p.converged) { publish_status("mpc_not_converged"); return; }
      trajectory_msgs::msg::JointTrajectory command;
      command.header.stamp = time; command.header.frame_id = base_frame_;
      command.joint_names = {yaw_joint_, pitch_joint_};
      for (int k = 0; k < mpc_config_.horizon; ++k) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {y.states[k][0], p.states[k][0]};
        point.velocities = {y.states[k][1], p.states[k][1]};
        point.accelerations = {y.accelerations[k], p.accelerations[k]};
        point.time_from_start = rclcpp::Duration::from_seconds((k + 1) * mpc_config_.dt);
        command.points.push_back(point);
      }
      trajectory_->publish(command);
      geometry_msgs::msg::PointStamped selected;
      selected.header.stamp = time; selected.header.frame_id = fixed_frame_;
      const auto target = tracker_->plate_at(index, t).position;
      selected.point.x = target.x(); selected.point.y = target.y(); selected.point.z = target.z();
      plate_->publish(selected);
      const auto x = tracker_->state_at(t);
      nav_msgs::msg::Odometry estimate;
      estimate.header = selected.header;
      // Both centre pose and velocity are expressed in fixed_frame (no child TF is broadcast).
      estimate.child_frame_id = fixed_frame_;
      estimate.pose.pose.position.x = x[0]; estimate.pose.pose.position.y = x[2]; estimate.pose.pose.position.z = x[4];
      estimate.pose.pose.orientation.z = std::sin(x[6] / 2); estimate.pose.pose.orientation.w = std::cos(x[6] / 2);
      estimate.twist.twist.linear.x = x[1]; estimate.twist.twist.linear.y = x[3]; estimate.twist.twist.linear.z = x[5];
      estimate.twist.twist.angular.z = x[7];
      const auto covariance = tracker_->covariance_at(t);
      const int pose_indices[]{0, 2, 4, -1, -1, 6};
      const int twist_indices[]{1, 3, 5, -1, -1, 7};
      for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) {
        if (pose_indices[i] >= 0 && pose_indices[j] >= 0)
          estimate.pose.covariance[6 * i + j] = covariance(pose_indices[i], pose_indices[j]);
        if (twist_indices[i] >= 0 && twist_indices[j] >= 0)
          estimate.twist.covariance[6 * i + j] = covariance(twist_indices[i], twist_indices[j]);
      }
      estimate.pose.covariance[21] = estimate.pose.covariance[28] = 1e6;
      estimate.twist.covariance[21] = estimate.twist.covariance[28] = 1e6;
      state_->publish(estimate);
      publish_status("tracking plate=" + std::to_string(index) + " radius_m=" + std::to_string(x[8]));
    } catch (const std::exception & e) {
      publish_status("planning_unavailable");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
    }
  }
  std::string fixed_frame_, base_frame_, camera_frame_, yaw_joint_, pitch_joint_, roll_joint_;
  double roll_{0.0};
  int64_t feedback_stamp_ns_{0};
  double timeout_{}, age_limit_{}, normal_sign_{}, feedback_stamp_{-1}, last_tick_{-1};
  double camera_offset_tolerance_{}, bullet_speed_{22.0}, gravity_{9.7833};
  Eigen::Vector2d yaw_{Eigen::Vector2d::Zero()}, pitch_{Eigen::Vector2d::Zero()};
  Eigen::Vector3d pitch_to_camera_{Eigen::Vector3d::Zero()},
    pitch_to_barrel_{0.0741, 0.0, 0.0};
  MpcConfig mpc_config_;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<AxisMpc> yaw_mpc_, pitch_mpc_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr observations_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bullet_speed_subscription_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr plate_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
};
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<rm_aa_control::ControlNode>()); }
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_control"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}


