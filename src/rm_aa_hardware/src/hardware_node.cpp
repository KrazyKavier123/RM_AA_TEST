#include "rm_aa_hardware/protocol.hpp"
#include "rm_aa_hardware/msg/gimbal_feedback.hpp"
#include "rm_aa_hardware/msg/target_command.hpp"
#include "rm_aa_hardware/SerialPort.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace rm_aa_hardware {
namespace {
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point t) { return std::chrono::duration<double>(t.time_since_epoch()).count(); }
double age(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }
}
class HardwareNode final : public rclcpp::Node {
public:
  HardwareNode() : Node("rm_aa_hardware") {
    if (!wire_platform_supported()) throw std::runtime_error("Protocol requires IEEE754 float32 and little-endian host");
    if (get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("Real serial hardware requires use_sim_time=false");
    device_ = declare_parameter<std::string>("serial.device", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("serial.baud_rate", 115200);
    const std::array<int, 9> supported_baud{300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
    if (std::find(supported_baud.begin(), supported_baud.end(), baud_) == supported_baud.end() && baud_ != 230400)
      throw std::invalid_argument("Unsupported serial.baud_rate");
    reconnect_ = positive("serial.reconnect_s", 1.0);
    tx_period_ = 1.0 / positive("serial.tx_rate_hz", 100.0);
    if (tx_period_ < 0.005) throw std::invalid_argument("serial.tx_rate_hz must be <= 200");
    feedback_timeout_ = positive("feedback_timeout_s", 0.15);
    command_timeout_ = positive("command_timeout_s", 0.15);
    source_ = declare_parameter<std::string>("command_source", "target_command");
    if (source_ != "target_command" && source_ != "trajectory")
      throw std::invalid_argument("command_source must be target_command or trajectory");
    trajectory_id_ = declare_parameter<int>("trajectory_target_id", 0);
    if (trajectory_id_ < 0 || trajectory_id_ > 255) throw std::invalid_argument("trajectory_target_id outside uint8 range");
    enable_fire_ = declare_parameter<bool>("enable_fire", false);
    base_frame_ = declare_parameter<std::string>("base_frame", "gimbal_base");
    attitude_frame_ = declare_parameter<std::string>("attitude_frame", "gimbal_attitude");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    joints_ = declare_parameter<std::vector<std::string>>("joint_names",
      {"gimbal_yaw_joint", "gimbal_pitch_joint", "gimbal_roll_joint"});
    if (device_.empty() || base_frame_.empty() || attitude_frame_.empty() || base_frame_ == attitude_frame_ ||
        joints_.size() != 3 || std::any_of(joints_.begin(), joints_.end(), [](const auto & s) { return s.empty(); }) ||
        joints_[0] == joints_[1] || joints_[0] == joints_[2] || joints_[1] == joints_[2])
      throw std::invalid_argument("Invalid frame/device/joint names");
    broadcast_attitude_ = declare_parameter<bool>("publish_attitude_tf", true);
    const bool camera_tf = declare_parameter<bool>("publish_camera_tf", false);
    const bool calibrated = declare_parameter<bool>("camera_extrinsics_calibrated", false);
    const bool fixed_base_tf = declare_parameter<bool>("publish_fixed_base_tf", false);
    const auto translation = declare_parameter<std::vector<double>>("camera_translation_m", {0.0, 0.0, 0.0});
    const auto quaternion = declare_parameter<std::vector<double>>("camera_quaternion_xyzw", {-0.5, 0.5, -0.5, 0.5});
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/gimbal/joint_states", rclcpp::SensorDataQoS());
    feedback_pub_ = create_publisher<msg::GimbalFeedback>("/gimbal/feedback", rclcpp::SensorDataQoS());
    connected_pub_ = create_publisher<std_msgs::msg::Bool>("/hardware/connected", 1);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    std::vector<geometry_msgs::msg::TransformStamped> statics;
    if (fixed_base_tf) {
      if (fixed_frame_.empty() || fixed_frame_ == base_frame_ || fixed_frame_ == attitude_frame_ || fixed_frame_ == camera_frame_)
        throw std::invalid_argument("Invalid fixed_frame for stationary-base TF");
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now(); t.header.frame_id = fixed_frame_; t.child_frame_id = base_frame_;
      t.transform.rotation.w = 1; statics.push_back(t);
    }
    if (camera_tf) {
      if (!calibrated || camera_frame_.empty() || camera_frame_ == base_frame_ || camera_frame_ == attitude_frame_ ||
          translation.size() != 3 || quaternion.size() != 4 ||
          !std::all_of(translation.begin(), translation.end(), [](double v) { return std::isfinite(v); }) ||
          !std::all_of(quaternion.begin(), quaternion.end(), [](double v) { return std::isfinite(v); }))
        throw std::invalid_argument("Camera TF requires measured translation/quaternion and calibrated flag");
      double norm = 0; for (double v : quaternion) norm += v*v;
      if (std::abs(norm - 1) > 1e-3) throw std::invalid_argument("Camera quaternion must be unit length");
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now(); t.header.frame_id = attitude_frame_; t.child_frame_id = camera_frame_;
      t.transform.translation.x = translation[0]; t.transform.translation.y = translation[1]; t.transform.translation.z = translation[2];
      t.transform.rotation.x = quaternion[0]; t.transform.rotation.y = quaternion[1];
      t.transform.rotation.z = quaternion[2]; t.transform.rotation.w = quaternion[3]; statics.push_back(t);
    }
    if (!statics.empty()) static_tf_->sendTransform(statics);
    if (source_ == "target_command") {
      command_sub_ = create_subscription<msg::TargetCommand>("/gimbal/target_command", 1,
        [this](msg::TargetCommand::ConstSharedPtr m) { receive_command(*m); });
    } else {
      trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>("/gimbal/trajectory", 1,
        [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr m) { receive_trajectory(*m); });
      if (trajectory_id_ == 0)
        RCLCPP_WARN(get_logger(), "trajectory_target_id=0: trajectories cannot identify a target; output remains no-target until configured");
    }
    parameters_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result; result.successful = parameters.empty();
      result.reason = "Hardware parameters are startup-only; edit YAML and restart"; return result;
    });
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { publish_feedback(); });
    worker_ = std::thread([this] { io_loop(); });  // Start last: constructor failures cannot leave a joinable thread.
  }
  ~HardwareNode() override {
    stop_.store(true); wake_.notify_all();
    if (worker_.joinable()) worker_.join();
  }
private:
  struct Received {
    MessData_AutoAim frame{};
    AttitudeSample attitude;
    builtin_interfaces::msg::Time stamp;
  };
  double positive(const std::string & key, double fallback) {
    const double value = declare_parameter<double>(key, fallback);
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(key + " must be finite and positive");
    return value;
  }
  bool command_header(const std_msgs::msg::Header & header, double & elapsed) {
    if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000U) return false;
    const rclcpp::Time stamp(header.stamp, get_clock()->get_clock_type());
    elapsed = (now() - stamp).seconds();
    return header.frame_id == base_frame_ && stamp.nanoseconds() > 0 && elapsed >= 0 && elapsed <= command_timeout_;
  }
  void receive_command(const msg::TargetCommand & m) {
    double elapsed = 0;
    Command c{m.target_yaw, m.target_pitch, m.yaw_velocity, m.pitch_velocity,
      m.distance_m, m.center_x_m, m.center_y_m, m.processing_latency_ms,
      m.target_id, m.tracking_state, m.fire_allowed};
    const bool valid = command_header(m.header, elapsed) && valid_command(c);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid) { have_command_ = false; return; }
    const rclcpp::Time stamp(m.header.stamp, get_clock()->get_clock_type());
    if (!connected_ || !have_feedback_ || age(feedback_at_) > feedback_timeout_ || stamp.nanoseconds() <= last_command_stamp_) return;
    command_ = c; command_age_ = elapsed; command_at_ = Clock::now();
    last_command_stamp_ = stamp.nanoseconds(); have_command_ = true;
  }
  void receive_trajectory(const trajectory_msgs::msg::JointTrajectory & m) {
    double elapsed = 0;
    bool valid = command_header(m.header, elapsed);
    std::array<std::size_t, 2> indices{};
    for (std::size_t axis = 0; axis < 2; ++axis) {
      const auto it = std::find(m.joint_names.begin(), m.joint_names.end(), joints_[axis]);
      if (it == m.joint_names.end() || std::count(m.joint_names.begin(), m.joint_names.end(), joints_[axis]) != 1) valid = false;
      indices[axis] = static_cast<std::size_t>(it - m.joint_names.begin());
    }
    std::vector<TrajectoryPoint> points;
    for (const auto & p : m.points) {
      if (p.positions.size() != m.joint_names.size() || p.velocities.size() != m.joint_names.size() || !valid ||
          p.time_from_start.sec < 0 || p.time_from_start.nanosec >= 1000000000U) { valid = false; break; }
      points.push_back({rclcpp::Duration(p.time_from_start).seconds(),
        {p.positions[indices[0]], p.positions[indices[1]]}, {p.velocities[indices[0]], p.velocities[indices[1]]}});
    }
    valid = valid && valid_trajectory(points) && elapsed <= points.back().time;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid || trajectory_id_ == 0) { have_command_ = false; return; }
    const rclcpp::Time stamp(m.header.stamp, get_clock()->get_clock_type());
    if (!connected_ || !have_feedback_ || !latest_attitude_.velocity_valid || age(feedback_at_) > feedback_timeout_ ||
        stamp.nanoseconds() <= last_command_stamp_) return;
    points_ = std::move(points);
    // Estimate the t=0 endpoint from the latest feedback; short receipt delays are bounded by the watchdog.
    for (std::size_t i = 0; i < 2; ++i) {
      initial_velocity_[i] = latest_attitude_.velocity[i];
      initial_position_[i] = latest_attitude_.position[i] - initial_velocity_[i] * elapsed;
    }
    command_ = {}; command_.target_id = static_cast<unsigned>(trajectory_id_); command_.tracking_state = 2;
    command_age_ = elapsed; command_at_ = Clock::now(); last_command_stamp_ = stamp.nanoseconds(); have_command_ = true;
  }
  void accept_frame(const Translator & f, AttitudeEstimator & estimator) {
    const auto received = Clock::now();
    auto attitude = estimator.update(f.message, seconds(received), feedback_timeout_);
    Received sample{f.message, attitude, static_cast<builtin_interfaces::msg::Time>(now())};
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_feedback_ || age(feedback_at_) > feedback_timeout_ || feedback_.status != f.message.status) have_command_ = false;
    feedback_ = f.message; latest_attitude_ = attitude; feedback_at_ = received; have_feedback_ = true;
    if (received_.size() == 32) received_.pop_front();
    received_.push_back(sample);
  }
  Translator next_output(bool stopping = false) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool fresh = have_feedback_ && age(feedback_at_) <= feedback_timeout_;
    const double command_elapsed = command_age_ + age(command_at_);
    Command candidate = command_;
    bool usable = !stopping && fresh && have_command_ && command_elapsed <= command_timeout_;
    if (usable && source_ == "trajectory") {
      usable = sample_trajectory(points_, command_elapsed, initial_position_, initial_velocity_, candidate);
      candidate.latency_ms = command_elapsed * 1000;
    } else if (usable) {
      // Preserve producer's measured processing latency, add time since its stamped output.
      candidate.latency_ms += command_elapsed * 1000;
    }
    return output_frame(feedback_, usable ? &candidate : nullptr, fresh, enable_fire_);
  }
  void io_loop() noexcept {
    while (!stop_.load()) {
      try {
        SerialPort port(device_);
        if (!port.isOpen() || !port.InitSerialPort(baud_, 8, 1, 'N')) throw std::runtime_error("Cannot open/configure " + device_);
        port.setWriteTimeout(std::chrono::milliseconds(20));
        FrameParser parser; AttitudeEstimator estimator;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          connected_ = true; have_feedback_ = have_command_ = false; last_command_stamp_ = 0; received_.clear();
        }
        RCLCPP_INFO(get_logger(), "Serial connected: %s at %d baud (64-byte RMVtraining protocol)", device_.c_str(), baud_);
        auto next_tx = Clock::now();
        while (!stop_.load()) {
          pollfd fd{port.ReceiveFd(), POLLIN, 0};
          const int ready = ::poll(&fd, 1, 5);
          if (ready < 0 && errno != EINTR) throw std::runtime_error("Serial poll failed");
          if (ready > 0 && (fd.revents & (POLLERR | POLLHUP | POLLNVAL))) throw std::runtime_error("Serial disconnected");
          if (ready > 0 && (fd.revents & POLLIN)) {
            char buffer[512];
            const auto count = ::read(port.ReceiveFd(), buffer, sizeof(buffer));
            if (count > 0) {
              for (const auto & frame : parser.append(buffer, static_cast<std::size_t>(count))) accept_frame(frame, estimator);
            } else if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
              throw std::runtime_error("Serial read failed");
            }
          }
          if (Clock::now() >= next_tx) {
            const auto packet = next_output();
            if (port.Write(packet.data, static_cast<int>(kCommunicationFrameSize)) != static_cast<int>(kCommunicationFrameSize))
              throw std::runtime_error("Serial write failed or timed out");
            next_tx = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(tx_period_));
          }
        }
        const auto stop_packet = next_output(true);
        port.Write(stop_packet.data, static_cast<int>(kCommunicationFrameSize));
      } catch (const std::exception & e) {
        if (!stop_.load()) RCLCPP_ERROR(get_logger(), "%s; retrying in %.2f s", e.what(), reconnect_);
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = have_feedback_ = have_command_ = false; received_.clear();
      }
      std::unique_lock<std::mutex> wait_lock(wait_mutex_);
      wake_.wait_for(wait_lock, std::chrono::duration<double>(reconnect_), [this] { return stop_.load(); });
    }
  }
  void publish_feedback() {
    std::deque<Received> batch;
    bool connected;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      batch.swap(received_); connected = connected_;
    }
    std_msgs::msg::Bool link; link.data = connected; connected_pub_->publish(link);
    for (const auto & sample : batch) {
      const auto & a = sample.attitude;
      msg::GimbalFeedback msg;
      msg.header.stamp = sample.stamp; msg.header.frame_id = base_frame_;
      msg.yaw = a.position[0]; msg.pitch = a.position[1]; msg.roll = a.position[2];
      msg.yaw_velocity = a.velocity[0]; msg.pitch_velocity = a.velocity[1]; msg.roll_velocity = a.velocity[2];
      msg.velocity_valid = a.velocity_valid; msg.status = sample.frame.status; msg.is_far = sample.frame.is_far;
      msg.prediction_bias_s = sample.frame.bias;
      if (broadcast_attitude_) {
        geometry_msgs::msg::TransformStamped t;
        t.header = msg.header; t.child_frame_id = attitude_frame_;
        const auto q = attitude_quaternion(msg.yaw, msg.pitch, msg.roll);
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1];
        t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]; tf_->sendTransform(t);
      }
      feedback_pub_->publish(msg);
      sensor_msgs::msg::JointState joints;
      joints.header = msg.header; joints.name = joints_;
      joints.position.assign(a.position.begin(), a.position.end());
      // Empty velocity on the first sample/gap: do not fabricate measured zero velocity.
      if (a.velocity_valid) joints.velocity.assign(a.velocity.begin(), a.velocity.end());
      joint_pub_->publish(joints);
    }
  }
  std::string device_, source_, base_frame_, attitude_frame_, camera_frame_, fixed_frame_;
  std::vector<std::string> joints_;
  int baud_{}, trajectory_id_{};
  double reconnect_{}, tx_period_{}, feedback_timeout_{}, command_timeout_{};
  bool enable_fire_{}, broadcast_attitude_{};
  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex mutex_, wait_mutex_;
  std::condition_variable wake_;
  bool connected_{}, have_feedback_{}, have_command_{};
  MessData_AutoAim feedback_{};
  AttitudeSample latest_attitude_;
  Clock::time_point feedback_at_{}, command_at_{};
  double command_age_{};
  int64_t last_command_stamp_{};
  Command command_;
  std::vector<TrajectoryPoint> points_;
  std::array<double, 2> initial_position_{}, initial_velocity_{};
  std::deque<Received> received_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<msg::GimbalFeedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Subscription<msg::TargetCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
};
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<rm_aa_hardware::HardwareNode>()); }
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("rm_aa_hardware"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
