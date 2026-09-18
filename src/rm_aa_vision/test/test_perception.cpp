#include <gtest/gtest.h>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "rm_aa_vision/preprocessor.hpp"
#include "rm_aa_vision/detector.hpp"
#include "rm_aa_vision/classifier.hpp"
#include "rm_aa_vision/pose_solver.hpp"
#include "rm_aa_vision/image_tracker.hpp"
#include "rm_aa_vision/source.hpp"

TEST(Preprocess, RejectsWrongImageType) {
  rm_aa::Preprocessor p;
  EXPECT_THROW(p.run(cv::Mat()), std::invalid_argument);
  EXPECT_THROW(p.run(cv::Mat::zeros(20,20,CV_8UC1)), std::invalid_argument);
}
TEST(Detection, FindsBothPlateSizesAndFiltersOwnColor) {
  auto image = rm_aa::synthetic_frame(1280,720,0);
  rm_aa::Preprocessor p;
  rm_aa::Detector d;
  const auto plates = d.match(d.find_lights(image, p.run(image)));
  ASSERT_EQ(plates.size(),2U);
  EXPECT_NE(plates[0].size, plates[1].size);
  for (const auto & plate : plates) {
    EXPECT_EQ(plate.color, rm_aa::Color::Blue);
    EXPECT_LT(plate.corners[0].x, plate.corners[1].x);
    EXPECT_LT(plate.corners[0].y, plate.corners[3].y);
  }
  rm_aa::DetectorConfig config; config.observed_color = rm_aa::Color::Red;
  rm_aa::Detector red(config);
  EXPECT_EQ(red.match(red.find_lights(image,p.run(image))).size(),1U);
}
TEST(Detection, EmptyImageHasNoPlate) {
  rm_aa::Preprocessor p; rm_aa::Detector d;
  const cv::Mat image = cv::Mat::zeros(200,300,CV_8UC3);
  EXPECT_TRUE(d.match(d.find_lights(image,p.run(image))).empty());
}
TEST(Pose, RecoversKnownPoseForBothSizes) {
  rm_aa::PoseSolver solver;
  rm_aa::Calibration camera; camera.calibrated = true;
  const cv::Vec3d rotation(0.08,0.2,0.02), translation(0.2,0.08,2.5);
  for (auto size : {rm_aa::PlateSize::Small, rm_aa::PlateSize::Large}) {
    std::vector<cv::Point2d> image_points;
    cv::projectPoints(solver.object_points(size),rotation,translation,camera.matrix,camera.distortion,image_points);
    rm_aa::Plate plate; plate.size = size;
    for (size_t i=0; i<4; ++i) plate.corners[i] = image_points[i];
    const auto pose = solver.solve(plate,camera);
    ASSERT_TRUE(pose.has_value());
    EXPECT_NEAR(cv::norm(pose->translation-translation),0,0.002);
    EXPECT_LT(pose->reprojection_rmse_px,0.01);
    EXPECT_GT(pose->bearing_yaw_rad,0);
    EXPECT_LT(pose->bearing_pitch_rad,0);
  }
}
TEST(Pose, NeverUsesUncalibratedCamera) {
  rm_aa::PoseSolver solver;
  EXPECT_FALSE(solver.solve(rm_aa::Plate{},rm_aa::Calibration{}).has_value());
}
TEST(Classifier, LoadsBundledModelAndProducesFiniteOutput) {
  rm_aa::Classifier classifier(std::string(MODEL_DIRECTORY)+"/mlp.onnx",std::string(MODEL_DIRECTORY)+"/label.txt",0.7);
  const auto image = rm_aa::synthetic_frame(1280,720,0);
  rm_aa::Preprocessor p; rm_aa::Detector d;
  auto plates = d.match(d.find_lights(image,p.run(image)));
  ASSERT_FALSE(plates.empty());
  EXPECT_EQ(classifier.extract(image,plates[0]).size(),cv::Size(20,28));
  classifier.classify(image,plates[0]);
  EXPECT_GE(plates[0].confidence,0); EXPECT_LE(plates[0].confidence,1);
  EXPECT_FALSE(plates[0].label.empty());
}
TEST(ImageTracking, EstimatesImageVelocityAndResetsAfterGap) {
  rm_aa::ImageTracker tracker;
  rm_aa::Plate p; p.size = rm_aa::PlateSize::Small;
  std::vector<rm_aa::ImageTrack> result;
  for (int i=0;i<100;++i) {
    p.center = {100.0F+i,200.0F};
    result = tracker.update({p},i*0.01+1);
  }
  ASSERT_EQ(result.size(),1U);
  EXPECT_NEAR(result[0].velocity_px_s.x,100,8);
  const int old_id = result[0].id;
  result = tracker.update({p},4.0);
  ASSERT_EQ(result.size(),1U);
  EXPECT_NE(result[0].id,old_id);
  EXPECT_FLOAT_EQ(result[0].velocity_px_s.x,0);
  EXPECT_GE(tracker.reset_count(),1);
}
TEST(ImageTracking, OneObservationCannotUpdateTwoTracks) {
  rm_aa::ImageTracker tracker;
  rm_aa::Plate a,b; a.center = {100,100}; b.center = {150,100};
  tracker.update({a,b},1.0);
  a.center = {120,100};
  auto result = tracker.update({a},1.02);
  ASSERT_EQ(result.size(),2U);
  EXPECT_NE(result[0].observed,result[1].observed);
  EXPECT_TRUE(tracker.update({},1.5).empty());
}
TEST(ImageTracking, ClockReversalClearsStateAndCovariance) {
  rm_aa::ImageTracker tracker;
  rm_aa::Plate p; p.center = {100,100};
  tracker.update({p},2.0);
  auto tracks = tracker.update({p},1.0);
  ASSERT_EQ(tracks.size(),1U);
  EXPECT_EQ(tracker.reset_count(),1);
  EXPECT_DOUBLE_EQ(tracks[0].filter.errorCovPost.at<double>(0,0),25);
}
