#include "rm_aa_vision/preprocessor.hpp"
#include "rm_aa_vision/detector.hpp"
#include "rm_aa_vision/classifier.hpp"
#include "rm_aa_vision/pose_solver.hpp"
#include "rm_aa_vision/image_tracker.hpp"
#include "rm_aa_vision/source.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <cmath>

namespace rm_aa {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
struct Frame {
  cv::Mat image;
  rclcpp::Time stamp;
  std::string frame_id;
  uint64_t sequence{};
};
class PerceptionNode final : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception") {
    mode_ = declare_parameter<std::string>("source.mode", "video");
    SourceConfig source;
    source.mode = mode_;
    source.device = declare_parameter<std::string>("source.device", "0");
    source.video_path = declare_parameter<std::string>("source.video_path", "");
    source.width = declare_parameter<int>("source.width", 1280);
    source.height = declare_parameter<int>("source.height", 720);
    source.fps = declare_parameter<double>("source.fps", 60.0);
    source.loop_video = declare_parameter<bool>("source.loop_video", false);
    source.camera_backend = declare_parameter<std::string>("source.camera_backend", "hikrobot");
    source.serial_number = declare_parameter<std::string>("source.serial_number", "");
    source.exposure_us = declare_parameter<double>("source.exposure_us", 5000.0);
    source.gain_db = declare_parameter<double>("source.gain_db", 0.0);
    source.grab_timeout_ms = declare_parameter<int>("source.grab_timeout_ms", 1000);
    frame_id_ = declare_parameter<std::string>("source.frame_id", "camera_optical_frame");
    const auto image_topic = declare_parameter<std::string>("source.image_topic", "/camera/image_raw");
    const auto info_topic = declare_parameter<std::string>("source.camera_info_topic", "/camera/camera_info");
    const int threads = declare_parameter<int>("opencv_threads", 2);
    cv::setNumThreads(std::max(1, threads));
    web_fps_ = declare_parameter<double>("debug.fps", 10.0);
    jpeg_quality_ = declare_parameter<int>("debug.jpeg_quality", 75);
    if (web_fps_ <= 0 || web_fps_ > 60 || jpeg_quality_ < 1 || jpeg_quality_ > 100)
      throw std::invalid_argument("Invalid debug FPS/JPEG quality");
    preprocess_ = std::make_unique<Preprocessor>(PreprocessConfig{
      declare_parameter<int>("preprocess.brightness_threshold", 100),
      declare_parameter<int>("preprocess.opening_radius", 0)});
    DetectorConfig detector;
    const auto own_color = declare_parameter<std::string>("detector.own_color", "red");
    if (own_color != "red" && own_color != "blue") throw std::invalid_argument("own_color must be red or blue");
    detector.observed_color = own_color == "red" ? Color::Blue : Color::Red;
    detector.min_light_length = declare_parameter<double>("detector.min_light_length", 8.0);
    detector.max_light_width_ratio = declare_parameter<double>("detector.max_light_width_ratio", 0.45);
    detector.max_tilt_deg = declare_parameter<double>("detector.max_tilt_deg", 35.0);
    detector.min_pair_ratio = declare_parameter<double>("detector.min_pair_ratio", 1.5);
    detector.max_pair_ratio = declare_parameter<double>("detector.max_pair_ratio", 5.0);
    detector.large_pair_ratio = declare_parameter<double>("detector.large_pair_ratio", 3.3);
    detector.min_length_ratio = declare_parameter<double>("detector.min_length_ratio", 0.65);
    detector.max_vertical_ratio = declare_parameter<double>("detector.max_vertical_ratio", 0.6);
    detector.min_color_difference = declare_parameter<double>("detector.min_color_difference", 8.0);
    detector_ = std::make_unique<Detector>(detector);
    const auto model_dir = ament_index_cpp::get_package_share_directory("rm_aa_vision") + "/models/";
    classifier_ = std::make_unique<Classifier>(
      declare_parameter<std::string>("classifier.model", model_dir + "mlp.onnx"),
      declare_parameter<std::string>("classifier.labels", model_dir + "label.txt"),
      declare_parameter<double>("classifier.confidence_threshold", 0.7));
    Dimensions dimensions;
    dimensions.small_width_m = declare_parameter<double>("dimensions.small_width_m", 0.135);
    dimensions.large_width_m = declare_parameter<double>("dimensions.large_width_m", 0.230);
    dimensions.height_m = declare_parameter<double>("dimensions.height_m", 0.057);
    solver_ = std::make_unique<PoseSolver>(dimensions, declare_parameter<double>("pnp.max_reprojection_px", 3.0));
    camera_.calibrated = declare_parameter<bool>("calibration.calibrated", false);
    camera_.width = declare_parameter<int>("calibration.width", 1280);
    camera_.height = declare_parameter<int>("calibration.height", 720);
    const auto k = declare_parameter<std::vector<double>>("calibration.k", {1000.0,0.0,640.0,0.0,1000.0,360.0,0.0,0.0,1.0});
    if (k.size() != 9) throw std::invalid_argument("calibration.k requires nine numbers");
    std::copy(k.begin(), k.end(), camera_.matrix.val);
    camera_.distortion = declare_parameter<std::vector<double>>("calibration.d", {0.0,0.0,0.0,0.0,0.0});
    validate_distortion(camera_.distortion);
    tracker_ = std::make_unique<ImageTracker>(
      declare_parameter<double>("image_tracker.association_px", 80.0),
      declare_parameter<double>("image_tracker.timeout_s", 0.3));
    parameter_callback_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result; result.successful = true;
      for (const auto & parameter : parameters) if (parameter.get_name() != "use_sim_time") {
        result.successful = false; result.reason = "Perception settings are startup-only; edit YAML and restart"; break;
      }
      return result;
    });
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    telemetry_ = create_publisher<std_msgs::msg::String>("telemetry", qos);
    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("debug/image/compressed", qos);
    mask_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("debug/mask/compressed", qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("observations", qos);
    if (mode_ == "topic") {
      info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
          {
            std::lock_guard<std::mutex> lock(camera_mutex_); camera_.calibrated = false;
          }
          if (info->binning_x > 1 || info->binning_y > 1 || info->roi.x_offset || info->roi.y_offset) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Provide intrinsics for the actual unbinned image"); return;
          }
          if (info->distortion_model != "plumb_bob" && info->distortion_model != "rational_polynomial") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported distortion model"); return;
          }
          try { validate_distortion(info->d); } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "%s", e.what()); return;
          }
          std::lock_guard<std::mutex> lock(camera_mutex_);
          std::copy(info->k.begin(), info->k.end(), camera_.matrix.val);
          camera_.distortion = info->d; camera_.width = info->width; camera_.height = info->height;
          camera_.calibrated = info->k[0] > 0 && info->k[4] > 0 && cv::checkRange(cv::Mat(camera_.matrix));
          calibration_frame_id_ = info->header.frame_id;
        });
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(image_topic, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          try {
            if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
              throw std::runtime_error("Image timestamp is zero; source must stamp images");
            enqueue(cv_bridge::toCvShare(msg, "bgr8")->image.clone(),
              rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()), msg->header.frame_id);
          } catch (const std::exception & e) { RCLCPP_WARN(get_logger(), "Image rejected: %s", e.what()); }
        });
    } else source_ = std::make_unique<Source>(source);
    worker_ = std::thread([this] { process_loop(); });
    if (source_) reader_ = std::thread([this] { capture_loop(); });
    RCLCPP_INFO(get_logger(), "Observation pipeline ready; source=%s, no actuator output", mode_.c_str());
  }
  ~PerceptionNode() override {
    running_ = false; ready_.notify_all();
    if (reader_.joinable()) reader_.join();
    if (worker_.joinable()) worker_.join();
  }
private:
  static void validate_distortion(const std::vector<double> & d) {
    if (!(d.empty() || d.size() == 4 || d.size() == 5 || d.size() == 8 || d.size() == 12 || d.size() == 14))
      throw std::invalid_argument("Unsupported distortion coefficient count");
    for (const auto x : d) if (!std::isfinite(x)) throw std::invalid_argument("Nonfinite distortion coefficient");
  }
  void enqueue(cv::Mat image, rclcpp::Time stamp, std::string id) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (pending_) ++dropped_;
    pending_ = Frame{std::move(image), stamp, std::move(id), ++captured_};
    ready_.notify_one();
  }
  void capture_loop() {
    auto next = Clock::now();
    while (running_ && rclcpp::ok()) {
      cv::Mat image;
      try {
        if (!source_->read(image)) {
          RCLCPP_WARN(get_logger(), "Capture ended or disconnected; dashboard will become stale"); break;
        }
        if (image.empty()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Camera grab timed out; retrying");
          continue;
        }
        enqueue(std::move(image), now(), frame_id_);
      } catch (const std::exception & e) { RCLCPP_ERROR(get_logger(), "Capture error: %s", e.what()); break; }
      if (!source_->realtime()) {
        next += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / source_->nominal_fps()));
        std::this_thread::sleep_until(next);
      }
    }
  }
  void process_loop() {
    auto last_debug = Clock::now() - std::chrono::seconds(1);
    auto rate_start = Clock::now();
    uint64_t rate_captured = 0, rate_processed = 0;
    double capture_fps = 0, processing_fps = 0;
    while (running_ && rclcpp::ok()) {
      std::optional<Frame> frame;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        ready_.wait_for(lock, std::chrono::milliseconds(100), [this] { return pending_.has_value() || !running_; });
        if (!running_) break;
        if (!pending_) continue;
        frame = std::move(pending_); pending_.reset();
      }
      try {
        const auto start = Clock::now();
        auto mask = preprocess_->run(frame->image);
        const auto lights = detector_->find_lights(frame->image, mask);
        auto plates = detector_->match(lights);
        for (auto & plate : plates) classifier_->classify(frame->image, plate);
        Calibration calibration;
        {
          std::lock_guard<std::mutex> lock(camera_mutex_); calibration = camera_;
          if (mode_ == "topic" && !calibration_frame_id_.empty() && frame->frame_id != calibration_frame_id_)
            calibration.calibrated = false;
        }
        const bool calibration_ok = calibration.calibrated && calibration.width == frame->image.cols &&
          calibration.height == frame->image.rows;
        calibration.calibrated = calibration_ok;
        const auto tracks = tracker_->update(plates, frame->stamp.seconds());
        geometry_msgs::msg::PoseArray poses;
        poses.header.stamp = frame->stamp; poses.header.frame_id = frame->frame_id;
        Json detections = Json::array();
        for (size_t i = 0; i < plates.size(); ++i) {
          const auto & plate = plates[i];
          Json item{{"index", i}, {"label", plate.label}, {"confidence", plate.confidence},
            {"accepted", plate.classification_valid}, {"size", size_name(plate.size)},
            {"color", color_name(plate.color)}, {"center_px", {plate.center.x, plate.center.y}}, {"pose", nullptr}};
          item["corners_px"] = Json::array();
          for (const auto & p : plate.corners) item["corners_px"].push_back({p.x, p.y});
          if (auto observation = solver_->solve(plate, calibration)) {
            item["pose"] = {{"position_m", {observation->translation[0], observation->translation[1], observation->translation[2]}},
              {"reprojection_rmse_px", observation->reprojection_rmse_px},
              {"bearing_yaw_rad", observation->bearing_yaw_rad}, {"bearing_pitch_rad", observation->bearing_pitch_rad}};
            geometry_msgs::msg::Pose pose;
            pose.position.x = observation->translation[0]; pose.position.y = observation->translation[1]; pose.position.z = observation->translation[2];
            const double angle = cv::norm(observation->rotation);
            const double scale = angle > 1e-10 ? std::sin(angle / 2) / angle : 0.5;
            pose.orientation.x = observation->rotation[0] * scale;
            pose.orientation.y = observation->rotation[1] * scale;
            pose.orientation.z = observation->rotation[2] * scale;
            pose.orientation.w = std::cos(angle / 2);
            item["pose_array_index"] = poses.poses.size(); poses.poses.push_back(pose);
          }
          detections.push_back(std::move(item));
        }
        pose_pub_->publish(poses); ++processed_;
        const auto end = Clock::now();
        const double elapsed = std::chrono::duration<double>(end - rate_start).count();
        if (elapsed >= 1) {
          const auto current_captured = captured_.load();
          capture_fps = (current_captured - rate_captured) / elapsed;
          processing_fps = (processed_ - rate_processed) / elapsed;
          rate_captured = current_captured; rate_processed = processed_; rate_start = end;
        }
        if (std::chrono::duration<double>(end - last_debug).count() < 1.0 / web_fps_) continue;
        last_debug = end;
        cv::Mat overlay = frame->image.clone();
        for (const auto & plate : plates) {
          std::vector<cv::Point> contour;
          for (const auto & p : plate.corners) contour.emplace_back(cvRound(p.x), cvRound(p.y));
          const cv::Scalar color = plate.classification_valid ? cv::Scalar(80,220,80) : cv::Scalar(0,190,255);
          cv::polylines(overlay, std::vector<std::vector<cv::Point>>{contour}, true, color, 2);
          const auto rect = cv::boundingRect(contour);
          cv::rectangle(overlay, rect, color, 1);
          cv::putText(overlay, plate.label + " " + size_name(plate.size), {rect.x, std::max(15, rect.y - 6)},
            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }
        Json track_data = Json::array();
        for (const auto & t : tracks) {
          track_data.push_back({{"id",t.id}, {"label",t.label}, {"observed",t.observed},
            {"position_px",{t.position.x,t.position.y}}, {"velocity_px_s",{t.velocity_px_s.x,t.velocity_px_s.y}}});
          cv::circle(overlay, t.position, 5, {255,220,50}, 1);
          cv::putText(overlay, "#" + std::to_string(t.id), t.position + cv::Point2f(8, 18),
            cv::FONT_HERSHEY_SIMPLEX, 0.45, {255,220,50}, 1);
        }
        publish_jpeg(image_pub_, overlay, poses.header);
        publish_jpeg(mask_pub_, mask, poses.header);
        const double frame_age_ms = (now() - frame->stamp).seconds() * 1000;
        Json status{{"schema",1}, {"mode",mode_}, {"sequence",frame->sequence},
          {"timestamp_s",frame->stamp.seconds()}, {"frame_id",frame->frame_id},
          {"calibration_valid",calibration_ok},
          {"capture_fps",capture_fps}, {"processing_fps",processing_fps},
          {"processing_ms",std::chrono::duration<double,std::milli>(end-start).count()},
          {"debug_ms",std::chrono::duration<double,std::milli>(Clock::now()-end).count()},
          {"frame_age_ms",frame_age_ms}, {"clock_valid",frame_age_ms >= 0},
          {"captured",captured_.load()}, {"processed",processed_}, {"dropped",dropped_.load()},
          {"light_count",lights.size()}, {"detections",detections}, {"image_tracks",track_data},
          {"tracker_resets",tracker_->reset_count()}, {"output_kind","observation_only"}};
        std_msgs::msg::String message; message.data = status.dump(); telemetry_->publish(message);
      } catch (const std::exception & e) {
        tracker_->reset();
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Frame rejected: %s", e.what());
      }
    }
  }
  void publish_jpeg(const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher,
    const cv::Mat & image, const std_msgs::msg::Header & header) {
    if (publisher->get_subscription_count() == 0) return;
    sensor_msgs::msg::CompressedImage message;
    message.header = header; message.format = image.channels() == 1 ? "mono8; jpeg compressed mono8" : "bgr8; jpeg compressed bgr8";
    if (cv::imencode(".jpg", image, message.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_})) publisher->publish(message);
  }
  std::string mode_, frame_id_, calibration_frame_id_;
  std::unique_ptr<Source> source_;
  std::unique_ptr<Preprocessor> preprocess_;
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<Classifier> classifier_;
  std::unique_ptr<PoseSolver> solver_;
  std::unique_ptr<ImageTracker> tracker_;
  Calibration camera_;
  std::mutex frame_mutex_, camera_mutex_;
  std::condition_variable ready_;
  std::optional<Frame> pending_;
  std::atomic<bool> running_{true};
  std::atomic<uint64_t> captured_{0}, dropped_{0};
  uint64_t processed_{0};
  double web_fps_{10}; int jpeg_quality_{75};
  std::thread reader_, worker_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_, mask_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rm_aa::PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  return 0;
}
