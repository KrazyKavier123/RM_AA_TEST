#include "StudentTasks.hpp"
#include "monitor.hpp"
#include "rm_aa_vision/source.hpp"
#include "rm_aa_vision/preprocessor.hpp"
#include "rm_aa_vision/detector.hpp"
#include "rm_aa_vision/classifier.hpp"
#include "rm_aa_vision/pose_solver.hpp"
#include "rm_aa_control/tracker.hpp"
#include "rm_aa_control/ballistic.hpp"
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

namespace {
template<class T> T setting(const cv::FileStorage& fs, const char* key, T fallback) {
  const auto node = fs[key];
  if (!node.empty()) node >> fallback;
  return fallback;
}
double positive(const cv::FileStorage& fs, const char* key, double fallback) {
  const double value = setting(fs, key, fallback);
  if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(std::string("Invalid ") + key);
  return value;
}
Eigen::Matrix3d gimbal_rotation(const GimbalState& g) {
  // Fixed/base axes: forward, left, up. Positive pitch lifts the muzzle.
  return (Eigen::AngleAxisd(g.yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(-g.pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(g.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}
Eigen::Vector3d eigen(const cv::Vec3d& v) { return {v[0], v[1], v[2]}; }
struct Pipeline {
  rm_aa::SourceConfig source_config;
  std::unique_ptr<rm_aa::Source> source;
  std::unique_ptr<rm_aa::Preprocessor> preprocess;
  std::unique_ptr<rm_aa::Classifier> classifier;
  rm_aa::DetectorConfig detector;
  rm_aa::PoseSolver solver{rm_aa::Dimensions{0.135, 0.230, 0.057}, 3.0};
  std::unique_ptr<rm_aa_control::Tracker> tracker;
  rm_aa_control::TrackerConfig tracker_config;
  int target_id{0}, updates{0}, stable_frames{10}, selected{-1};
  int outpost_id{0}, guard_id{0}, base_id{0};
  bool enable_fire{false}, use_ballistics{true};
  double speed{22}, gravity{9.7833}, max_prediction{0.8}, max_age{0.2};
  double fire_yaw{0.02}, fire_pitch{0.02}, normal_sign{1};
  double stable_position_variance{0.01}, stable_yaw_variance{0.1};
  double last_stamp{-1}, next_capture{0};
  Eigen::Vector3d muzzle{0.0741, 0, 0}, camera_offset{0, 0, 0};
  void reset() { tracker->reset(); target_id = updates = 0; selected = -1; }
  int id_for(const std::string& label) const {
    if (label.size() == 1 && label[0] >= '1' && label[0] <= '5') return label[0] - '0';
    if (label == "outpost") return outpost_id;
    if (label == "guard") return guard_id;
    if (label == "base") return base_id;
    return 0;
  }
};
std::unique_ptr<Pipeline> pipeline;
Pipeline& active() {
  if (!pipeline) throw std::logic_error("initialize_tasks must be called first");
  return *pipeline;
}
}

double monotonic_seconds() noexcept {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void initialize_tasks(const std::string& config_path, const CameraParameters& camera) {
  if (!camera.valid()) throw std::invalid_argument("Measured intrinsic/extrinsic calibration required");
  cv::FileStorage fs(config_path, cv::FileStorage::READ);
  if (!fs.isOpened()) throw std::runtime_error("Cannot read training config");
  auto p = std::make_unique<Pipeline>();
  auto& s = p->source_config;
  s.mode = setting<std::string>(fs, "source_mode", "camera");
  if (s.mode != "camera" && s.mode != "video" && s.mode != "synthetic")
    throw std::invalid_argument("source_mode must be camera, video or synthetic");
  s.camera_backend = setting<std::string>(fs, "camera_backend", "hikrobot");
  s.device = setting<std::string>(fs, "camera_device", "0");
  s.serial_number = setting<std::string>(fs, "camera_serial", "");
  s.width = camera.image_width; s.height = camera.image_height;
  s.fps = positive(fs, "camera_fps", 60);
  s.exposure_us = positive(fs, "exposure_us", 5000);
  s.gain_db = setting(fs, "gain_db", 0.0);
  s.grab_timeout_ms = setting(fs, "grab_timeout_ms", 100);
  auto path = [&](const std::string& value) {
    const auto file = std::filesystem::path(value);
    return (file.is_absolute() ? file : std::filesystem::path(config_path).parent_path() / file).lexically_normal().string();
  };
  s.video_path = path(setting<std::string>(fs, "video_path", ""));
  s.loop_video = setting(fs, "loop_video", 0) != 0;
  p->preprocess = std::make_unique<rm_aa::Preprocessor>(rm_aa::PreprocessConfig{
    setting(fs, "brightness_threshold", 100), setting(fs, "opening_radius", 0)});
  p->classifier = std::make_unique<rm_aa::Classifier>(
    path(setting<std::string>(fs, "classifier_model", "../../src/rm_aa_vision/models/mlp.onnx")),
    path(setting<std::string>(fs, "classifier_labels", "../../src/rm_aa_vision/models/label.txt")),
    setting(fs, "confidence_threshold", 0.7));
  p->solver = rm_aa::PoseSolver({0.135, 0.230, 0.057}, positive(fs, "max_reprojection_px", 3));
  auto& t = p->tracker_config;
  t.alternating_geometry = true;
  t.plate_count = setting(fs, "plate_count", 4);
  t.radius = positive(fs, "initial_radius_m", 0.25);
  t.timeout = positive(fs, "tracker_timeout_s", 0.3);
  t.max_height_difference = positive(fs, "max_height_difference_m", 0.3);
  p->tracker = std::make_unique<rm_aa_control::Tracker>(t);
  p->stable_frames = setting(fs, "stable_frames", 10);
  if (p->stable_frames < 2) throw std::invalid_argument("stable_frames must be >= 2");
  p->stable_position_variance = positive(fs, "stable_position_variance", 0.01);
  p->stable_yaw_variance = positive(fs, "stable_yaw_variance", 0.1);
  p->max_age = positive(fs, "max_processing_age_s", 0.2);
  p->max_prediction = positive(fs, "max_prediction_s", 0.8);
  p->speed = positive(fs, "bullet_speed_mps", 22);
  p->gravity = positive(fs, "gravity_mps2", 9.7833);
  p->use_ballistics = setting(fs, "use_ballistics", 1) != 0;
  p->enable_fire = setting(fs, "enable_fire", 0) != 0;
  p->fire_yaw = positive(fs, "fire_yaw_tolerance_rad", 0.02);
  p->fire_pitch = positive(fs, "fire_pitch_tolerance_rad", 0.02);
  p->normal_sign = setting(fs, "plate_normal_sign", 1.0);
  if (p->normal_sign != 1 && p->normal_sign != -1) throw std::invalid_argument("Invalid plate_normal_sign");
  p->outpost_id = setting(fs, "outpost_id", 0); p->guard_id = setting(fs, "guard_id", 0); p->base_id = setting(fs, "base_id", 0);
  for (int id : {p->outpost_id, p->guard_id, p->base_id})
    if (id < 0 || id > 255) throw std::invalid_argument("Target ID outside uint8 range");
  cv::Mat muzzle;
  fs["pitch_to_barrel_m"] >> muzzle;
  if (!muzzle.empty()) {
    muzzle.convertTo(muzzle, CV_64F);
    if (muzzle.rows != 3 || muzzle.cols != 1 || !cv::checkRange(muzzle))
      throw std::invalid_argument("pitch_to_barrel_m must be a finite 3x1 matrix");
    for (int i = 0; i < 3; ++i) p->muzzle[i] = muzzle.at<double>(i);
  }
  for (int i = 0; i < 3; ++i) p->camera_offset[i] = camera.translation_camera_to_gimbal.at<double>(i);
  pipeline = std::move(p);
}
void shutdown_tasks() noexcept { pipeline.reset(); }
void reset_tracking() noexcept { if (pipeline) pipeline->reset(); }
bool prediction_is_fresh(double stamp) noexcept {
  const double age = monotonic_seconds() - stamp;
  return pipeline && std::isfinite(age) && age >= 0 && age <= pipeline->max_age;
}

bool get_pic(cv::Mat& pic) {
  auto& p = active();
  pic.release();
  try {
    if (!p.source) p.source = std::make_unique<rm_aa::Source>(p.source_config);
    if (!p.source->realtime()) {
      const double wait = p.next_capture - monotonic_seconds();
      if (wait > 0) std::this_thread::sleep_for(std::chrono::duration<double>(wait));
      p.next_capture = monotonic_seconds() + 1.0 / p.source->nominal_fps();
    }
    if (!p.source->read(pic) || pic.empty()) return false;
    if (pic.type() != CV_8UC3 || pic.cols != p.source_config.width || pic.rows != p.source_config.height)
      throw std::runtime_error("Frame type/size differs from measured calibration");
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Camera: " << e.what() << '\n';
    pic.release(); return false;
  }
}

std::vector<ArmorDetection> armor_detect(const cv::Mat& pic, TeamColor enemy_color) {
  auto& p = active();
  if (pic.empty()) return {};
  auto cfg = p.detector;
  cfg.observed_color = enemy_color == TeamColor::Red ? rm_aa::Color::Red : rm_aa::Color::Blue;
  rm_aa::Detector detector(cfg);
  auto plates = detector.match(detector.find_lights(pic, p.preprocess->run(pic)));
  std::vector<ArmorDetection> result;
  for (auto& plate : plates) {
    bool valid = true;
    for (const auto& c : plate.corners)
      valid &= std::isfinite(c.x) && std::isfinite(c.y) && c.x >= 0 && c.y >= 0 && c.x < pic.cols && c.y < pic.rows;
    if (!valid) continue;
    p.classifier->classify(pic, plate);
    const int id = p.id_for(plate.label);
    if (!plate.classification_valid || id == 0) continue;
    result.push_back({plate.corners, plate.size == rm_aa::PlateSize::Small ? ArmorSize::Small : ArmorSize::Large,
                      id, static_cast<float>(plate.confidence)});
  }
  return result;
}

std::vector<ArmorPose> armor_solve(const std::vector<ArmorDetection>& detections,
                                  const CameraParameters& camera, const GimbalState&) {
  if (!camera.valid()) return {};
  rm_aa::Calibration c;
  c.calibrated = true; c.width = camera.image_width; c.height = camera.image_height;
  for (int i = 0; i < 9; ++i) c.matrix.val[i] = camera.camera_matrix.at<double>(i / 3, i % 3);
  c.distortion.assign(camera.distortion_coefficients.ptr<double>(),
                      camera.distortion_coefficients.ptr<double>() + camera.distortion_coefficients.total());
  std::vector<ArmorPose> result;
  for (const auto& detection : detections) {
    if (detection.target_id <= 0 || !std::isfinite(detection.confidence)) continue;
    rm_aa::Plate plate;
    plate.corners = detection.corners;
    plate.size = detection.size == ArmorSize::Small ? rm_aa::PlateSize::Small : rm_aa::PlateSize::Large;
    const auto pose = active().solver.solve(plate, c);
    if (!pose) continue;
    cv::Mat position = camera.rotation_camera_to_gimbal * cv::Mat(pose->translation) + camera.translation_camera_to_gimbal;
    cv::Mat rotation;
    cv::Rodrigues(pose->rotation, rotation);
    const cv::Mat inward = active().normal_sign * camera.rotation_camera_to_gimbal * rotation.col(2);
    if (std::hypot(inward.at<double>(0), inward.at<double>(1)) < 0.2) continue;
    result.push_back({detection, pose->translation,
      {position.at<double>(0), position.at<double>(1), position.at<double>(2)},
      std::atan2(inward.at<double>(1), inward.at<double>(0)), pose->reprojection_rmse_px,
      {inward.at<double>(0), inward.at<double>(1), inward.at<double>(2)}});
  }
  return result;
}

PredictionResult ekf_predict(const std::vector<ArmorPose>& observations,
                             const GimbalState& gimbal, double stamp) {
  auto& p = active();
  const double now = monotonic_seconds(), age = now - stamp;
  if (!std::isfinite(stamp) || !std::isfinite(gimbal.yaw) || !std::isfinite(gimbal.pitch) ||
      !std::isfinite(gimbal.roll) || !std::isfinite(gimbal.prediction_bias_s) || age < 0 || age > p.max_age ||
      stamp <= p.last_stamp) { p.reset(); p.last_stamp = stamp; return {}; }
  p.last_stamp = stamp;
  const auto rotation = gimbal_rotation(gimbal);
  const Eigen::Vector3d camera = rotation * p.camera_offset;
  if (!p.tracker->valid(stamp)) p.reset();
  if (!p.target_id) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& o : observations) {
      const double d = cv::norm(o.position_camera_m);
      if (std::isfinite(d) && o.detection.target_id > 0 && o.detection.target_id <= 255 && d < best) {
        best = d; p.target_id = o.detection.target_id;
      }
    }
  }
  std::vector<rm_aa_control::Observation> matching;
  for (const auto& o : observations) {
    if (o.detection.target_id != p.target_id) continue;
    const Eigen::Vector3d normal = rotation * eigen(o.normal_gimbal);
    if (!normal.allFinite() || normal.head<2>().norm() < 0.2) continue;
    matching.push_back({rotation * eigen(o.position_gimbal_m), std::atan2(normal.y(), normal.x())});
  }
  const bool corrected = p.tracker->update(matching, stamp, camera);
  p.updates = corrected ? p.updates + 1 : 0;
  if (!p.tracker->valid(now) || !p.target_id) return {};
  const double lead = age + gimbal.prediction_bias_s;
  if (!std::isfinite(lead) || lead < 0 || lead > p.max_prediction) return {};
  const double fire_stamp = stamp + lead;
  int best = 0; double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < p.tracker_config.plate_count; ++i) {
    const double d = (p.tracker->plate_at(i, fire_stamp).position - camera).norm();
    if (d < distance) { best = i; distance = d; }
  }
  if (p.selected < 0 || distance + p.tracker_config.switch_margin <
      (p.tracker->plate_at(p.selected, fire_stamp).position - camera).norm()) p.selected = best;
  auto aim = [&](double time) {
    auto direct = [&](double hit) {
      const auto target = p.tracker->plate_at(p.selected, hit).position;
      if (p.use_ballistics) return rm_aa_control::ballistic_pointing_angles(target, p.muzzle, p.speed, p.gravity);
      return rm_aa_control::BallisticSolution{{std::atan2(target.y(), target.x()), std::atan2(target.z(), target.head<2>().norm())}, 0.0};
    };
    auto solution = direct(time);
    bool converged = !p.use_ballistics;
    for (int i = 0; i < 8 && !converged; ++i) {
      if (time + solution.flight_time - stamp > p.max_prediction) throw std::runtime_error("Prediction horizon exceeded");
      const auto next = direct(time + solution.flight_time);
      converged = std::abs(next.flight_time - solution.flight_time) < 1e-6;
      solution = next;
    }
    if (!converged) throw std::runtime_error("Intercept time did not converge");
    return solution;
  };
  try {
    const auto solution = aim(fire_stamp);
    const auto later = aim(fire_stamp + 0.01);
    const auto state = p.tracker->state_at(now);
    const auto covariance = p.tracker->covariance_at(now);
    const bool stable = corrected && p.updates >= p.stable_frames &&
      covariance(0,0) < p.stable_position_variance && covariance(2,2) < p.stable_position_variance &&
      covariance(4,4) < p.stable_position_variance && covariance(6,6) < p.stable_yaw_variance;
    PredictionResult result;
    result.state = stable ? TrackingState::Stable : TrackingState::Unstable;
    result.target_id = static_cast<std::uint8_t>(p.target_id);
    result.center_x_m = state[0]; result.center_y_m = state[2];
    result.center_velocity_x_m_s = state[1]; result.center_velocity_y_m_s = state[3];
    result.body_yaw = state[6]; result.body_yaw_velocity = state[7];
    result.radius_1_m = state[8]; result.radius_2_m = state[9];
    result.target_yaw = solution.angles[0]; result.target_pitch = solution.angles[1];
    result.target_yaw_velocity = rm_aa_control::wrap(later.angles[0] - solution.angles[0]) / 0.01;
    result.target_pitch_velocity = (later.angles[1] - solution.angles[1]) / 0.01;
    result.distance_m = (p.tracker->plate_at(p.selected, fire_stamp + solution.flight_time).position - camera).norm();
    result.fire_allowed = p.enable_fire && stable && std::abs(rm_aa_control::wrap(result.target_yaw - gimbal.yaw)) <= p.fire_yaw &&
      std::abs(result.target_pitch - gimbal.pitch) <= p.fire_pitch;
    monitor::log("ekf_z_m", state[4]); monitor::log("ekf_vz_m_s", state[5]); monitor::log("ekf_dz_m", state[10]);
    monitor::log("prediction_lead_s", lead); monitor::log("flight_time_s", solution.flight_time);
    return result;
  } catch (const std::exception& e) {
    std::cerr << "Prediction: " << e.what() << '\n';
    return {};
  }
}
