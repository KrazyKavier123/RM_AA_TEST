#include "StudentTasks.hpp"
#include "AppSupport.hpp"
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>
#include <filesystem>
#include <thread>
#include <chrono>

class TrainingPipeline : public ::testing::Test {
protected:
  CameraParameters camera;
  std::filesystem::path config;
  void SetUp() override {
    camera.calibrated = true;
    camera.image_width = 1280; camera.image_height = 720;
    camera.camera_matrix = (cv::Mat_<double>(3,3) << 1000,0,640,0,1000,360,0,0,1);
    camera.distortion_coefficients = cv::Mat::zeros(1,5,CV_64F);
    camera.rotation_camera_to_gimbal = (cv::Mat_<double>(3,3) << 0,0,1,-1,0,0,0,-1,0);
    camera.translation_camera_to_gimbal = (cv::Mat_<double>(3,1) << 0.02,-0.01,0.03);
    camera.mean_reprojection_error_px = 0.1;
    config = std::filesystem::current_path() /
      (std::string("training-test-") + ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".yaml");
    configure(false);
  }
  void configure(bool ballistics) {
    cv::FileStorage fs(config.string(), cv::FileStorage::WRITE);
    fs << "classifier_model" << std::string(MODEL_DIRECTORY) + "/mlp.onnx";
    fs << "classifier_labels" << std::string(MODEL_DIRECTORY) + "/label.txt";
    fs << "source_mode" << "synthetic" << "stable_frames" << 3 << "use_ballistics" << int(ballistics);
    fs << "stable_position_variance" << 0.03;
    fs << "enable_fire" << 1 << "fire_yaw_tolerance_rad" << 1.0 << "fire_pitch_tolerance_rad" << 1.0;
    fs.release();
    initialize_tasks(config.string(), camera);
  }
  void TearDown() override { shutdown_tasks(); std::filesystem::remove(config); }
};

TEST_F(TrainingPipeline, IntrinsicsExtrinsicsAndBothSpecifiedPlateSizes) {
  for (auto size : {ArmorSize::Small, ArmorSize::Large}) {
    const double w = size == ArmorSize::Small ? 0.135 : 0.230, h = 0.057;
    std::vector<cv::Point3d> object{{-w/2,-h/2,0},{w/2,-h/2,0},{w/2,h/2,0},{-w/2,h/2,0}};
    const cv::Vec3d r(0.08,0.15,0.02), t(0.1,0.05,3.0);
    std::vector<cv::Point2d> points;
    cv::projectPoints(object,r,t,camera.camera_matrix,camera.distortion_coefficients,points);
    ArmorDetection detection;
    detection.size = size; detection.target_id = 3; detection.confidence = 0.95F;
    for (int i=0;i<4;++i) detection.corners[i] = points[i];
    const auto observations = armor_solve({detection},camera,{});
    ASSERT_EQ(observations.size(),1U);
    EXPECT_LT(cv::norm(observations[0].position_camera_m-t), 0.002);
    EXPECT_LT(cv::norm(observations[0].position_gimbal_m-cv::Vec3d(3.02,-0.11,-0.02)),0.002);
  }
  camera.calibrated = false;
  EXPECT_TRUE(armor_solve({ArmorDetection{}},camera,{}).empty());
}

TEST_F(TrainingPipeline, SourceAndNegativeImage) {
  cv::Mat image;
  ASSERT_TRUE(get_pic(image));
  EXPECT_EQ(image.type(), CV_8UC3);
  EXPECT_EQ(image.size(), cv::Size(1280,720));
  EXPECT_TRUE(armor_detect(cv::Mat::zeros(720,1280,CV_8UC3),TeamColor::Blue).empty());
}

TEST_F(TrainingPipeline, BallisticLeadAndInvalidBias) {
  configure(true);
  ArmorPose observation;
  observation.detection.target_id = 3;
  observation.detection.confidence = 0.95F;
  observation.position_camera_m = {0,0,3};
  observation.position_gimbal_m = {3,0,0};
  observation.normal_gimbal = {1,0,0};
  const auto result = ekf_predict({observation}, {}, monotonic_seconds());
  ASSERT_EQ(result.state, TrackingState::Unstable);
  EXPECT_NEAR(result.target_yaw, 0.0, 1e-5);
  EXPECT_GT(result.target_pitch, 0.02);
  EXPECT_LT(result.target_pitch, 0.1);
  EXPECT_NEAR(result.distance_m, cv::norm(cv::Vec3d(2.98,0.01,-0.03)), 0.01);
  EXPECT_FALSE(result.fire_allowed);
  GimbalState invalid;
  invalid.prediction_bias_s = 2.0F;
  EXPECT_EQ(ekf_predict({observation}, invalid, monotonic_seconds()).state, TrackingState::NoTarget);
  invalid.prediction_bias_s = -1.0F;
  EXPECT_EQ(ekf_predict({observation}, invalid, monotonic_seconds()).state, TrackingState::NoTarget);
}

TEST_F(TrainingPipeline, GimbalMotionDoesNotMoveStaticTargetAndLossInhibitsFire) {
  PredictionResult result;
  for (int i=0;i<30;++i) {
    GimbalState g;
    g.yaw = 0.05F * i; g.pitch = 0.05F; g.roll = -0.03F; g.prediction_bias_s = 0.04F;
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(g.yaw,Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-g.pitch,Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(g.roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d position = rotation.transpose()*Eigen::Vector3d(2.75,0.2,0.1);
    const Eigen::Vector3d normal = rotation.transpose()*Eigen::Vector3d::UnitX();
    ArmorPose o;
    o.detection.target_id=3; o.detection.confidence=0.95F;
    o.position_camera_m={0,0,3};
    o.position_gimbal_m={position.x(),position.y(),position.z()};
    o.normal_gimbal={normal.x(),normal.y(),normal.z()};
    o.armor_yaw=std::atan2(normal.y(),normal.x());
    result=ekf_predict({o},g,monotonic_seconds());
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_EQ(result.state,TrackingState::Stable);
  EXPECT_NEAR(result.center_x_m,3,0.02);
  EXPECT_NEAR(result.center_y_m,0.2,0.02);
  EXPECT_NEAR(result.center_velocity_x_m_s,0,0.05);
  EXPECT_NEAR(result.center_velocity_y_m_s,0,0.05);
  result=ekf_predict({}, {}, monotonic_seconds());
  EXPECT_EQ(result.state,TrackingState::Unstable);
  EXPECT_FALSE(result.fire_allowed);
  result=ekf_predict({}, {}, monotonic_seconds()-1.0);
  EXPECT_EQ(result.state,TrackingState::NoTarget);
}
