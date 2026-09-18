#include "rm_aa_control/tracker.hpp"
#include "rm_aa_control/mpc.hpp"
#include "rm_aa_control/pointing.hpp"
#include "rm_aa_control/ballistic.hpp"
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <limits>

using namespace rm_aa_control;
TEST(Pointing, ZeroOffsetMatchesBearing) {
  const Eigen::Vector3d target(3, -1, 0.8);
  const auto angles = camera_pointing_angles(target, Eigen::Vector3d::Zero());
  EXPECT_NEAR(angles.x(), std::atan2(-1.0, 3.0), 1e-12);
  EXPECT_NEAR(angles.y(), std::atan2(0.8, std::sqrt(10.0)), 1e-12);
}
TEST(Pointing, RecoversCameraRayWithRotatingOffset) {
  // Construct targets by forward kinematics, then check inverse pointing.
  for (const Eigen::Vector3d offset : {Eigen::Vector3d(-0.06711, 0, -0.0515),
                                     Eigen::Vector3d(-0.0515, 0.06711, 0)}) {
    for (double yaw : {-3.12, -0.8, 0.0, 1.2, 3.12}) {
      for (double pitch : {-0.7, 0.0, 0.6}) {
        for (double distance : {0.2, 1.0, 5.0}) {
          const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
          const Eigen::Vector3d target = rotation * (offset + distance * Eigen::Vector3d::UnitX());
          const auto angles = camera_pointing_angles(target, offset);
          EXPECT_NEAR(wrap(angles.x() - yaw), 0, 1e-12);
          EXPECT_NEAR(angles.y(), pitch, 1e-12);
        }
      }
    }
  }
}
TEST(Pointing, RejectsInvalidAndUnreachableGeometry) {
  EXPECT_THROW(camera_pointing_angles({0, 0, 1}, {0, 0, 0}), std::runtime_error);
  EXPECT_THROW(camera_pointing_angles({0.01, 0, 0}, {0, 0.02, 0}), std::runtime_error);
  EXPECT_THROW(camera_pointing_angles({0.01, 0, 0}, {0, 0, 0.02}), std::runtime_error);
  EXPECT_THROW(camera_pointing_angles({0.01, 0, 0}, {0.02, 0, 0}), std::runtime_error);
  EXPECT_THROW(camera_pointing_angles({NAN, 0, 0}, {0, 0, 0}), std::invalid_argument);
}
namespace {
Observation moving_plate(double t, int index) {
  const double yaw = wrap(0.3 + 0.7 * t + index * pi / 2);
  return {{3 + 0.2 * t - 0.25 * std::cos(yaw),
           0.4 - 0.1 * t - 0.25 * std::sin(yaw), 0.2}, yaw};
}
}
TEST(Tracker, EstimatesMovingRotatingVehicleAcrossPlateChanges) {
  Tracker tracker;
  for (int k = 0; k < 600; ++k) {
    const double t = k * 0.02;
    // Changing visible plate must not change the estimated vehicle centre.
    ASSERT_TRUE(tracker.update({moving_plate(t, (k / 40) % 4)}, t, Eigen::Vector3d::Zero()));
  }
  const auto x = tracker.state_at(11.98);
  EXPECT_NEAR(x[0], 3 + 0.2 * 11.98, 0.04);
  EXPECT_NEAR(x[2], 0.4 - 0.1 * 11.98, 0.04);
  EXPECT_NEAR(x[1], 0.2, 0.05); EXPECT_NEAR(x[3], -0.1, 0.05);
  EXPECT_NEAR(x[7], 0.7, 0.06); EXPECT_NEAR(x[8], 0.25, 0.04);
  Eigen::SelfAdjointEigenSolver<Tracker::Covariance> eigen(tracker.covariance());
  EXPECT_GT(eigen.eigenvalues().minCoeff(), -1e-10);
}
TEST(Tracker, RejectsOutliersOldFramesAndExpires) {
  Tracker tracker;
  ASSERT_TRUE(tracker.update({moving_plate(1, 0)}, 1, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(tracker.update({{{100, 100, 100}, 0}}, 1.02, Eigen::Vector3d::Zero()));
  const auto x = tracker.state_at(1.02);
  EXPECT_FALSE(tracker.update({moving_plate(0.9, 0)}, 0.9, Eigen::Vector3d::Zero()));
  EXPECT_TRUE(x.isApprox(tracker.state_at(1.02)));
  EXPECT_TRUE(tracker.valid(1.2)); EXPECT_FALSE(tracker.valid(1.31));
  EXPECT_TRUE(tracker.update({moving_plate(2, 2)}, 2, Eigen::Vector3d::Zero()));
  tracker.reset(); EXPECT_FALSE(tracker.valid(2));
}
TEST(Tracker, EmptyAndNonfiniteDoNotInitialize) {
  Tracker tracker;
  EXPECT_FALSE(tracker.update({}, 1, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(tracker.update({{{1, 0, 0}, std::numeric_limits<double>::quiet_NaN()}}, 1, Eigen::Vector3d::Zero()));
  EXPECT_EQ(tracker.select_nearest(1, Eigen::Vector3d::Zero()), -1);
}
TEST(Tracker, NearestUsesCameraPositionAndHysteresis) {
  TrackerConfig config; config.switch_margin = 0.05;
  Tracker tracker(config);
  ASSERT_TRUE(tracker.update({{{2.75, 0, 0}, 0}}, 1, Eigen::Vector3d::Zero()));
  EXPECT_EQ(tracker.select_nearest(1, Eigen::Vector3d::Zero()), 0);
  EXPECT_EQ(tracker.select_nearest(1, {3, 0, 0}), 0);  // tie: keep the current plate
  EXPECT_EQ(tracker.select_nearest(1, {6, 0, 0}), 2);
}
TEST(Tracker, WrapsHeadingInnovation) {
  Tracker tracker;
  ASSERT_TRUE(tracker.update({{{3.25, 0, 0}, pi - 0.001}}, 1, Eigen::Vector3d::Zero()));
  ASSERT_TRUE(tracker.update({{{3.25, 0, 0}, -pi + 0.001}}, 1.02, Eigen::Vector3d::Zero()));
  EXPECT_LT(std::abs(wrap(tracker.state_at(1.02)[6] - pi)), 0.01);
}
TEST(Mpc, RespectsAccelerationAndDoubleIntegratorDynamics) {
  MpcConfig config;
  AxisMpc mpc(config);
  const Eigen::Vector2d start(0.2, -0.1);
  const auto plan = mpc.solve(start, std::vector<Eigen::Vector2d>(config.horizon, {1, 0}));
  ASSERT_TRUE(plan.converged);
  Eigen::Vector2d previous = start;
  for (int k = 0; k < config.horizon; ++k) {
    const double u = plan.accelerations[k];
    EXPECT_LE(std::abs(u), config.max_acceleration + 1e-10);
    EXPECT_NEAR(plan.states[k][0], previous[0] + config.dt * previous[1] + 0.5 * config.dt * config.dt * u, 1e-10);
    EXPECT_NEAR(plan.states[k][1], previous[1] + config.dt * u, 1e-10);
    previous = plan.states[k];
  }
  EXPECT_GT(previous[0], start[0]);
}
TEST(Mpc, ClosedLoopSettlesFromMeasuredState) {
  MpcConfig config; AxisMpc mpc(config);
  Eigen::Vector2d measured(0, 0);
  for (int k = 0; k < 250; ++k) {
    const auto plan = mpc.solve(measured, std::vector<Eigen::Vector2d>(config.horizon, {0.8, 0}));
    ASSERT_TRUE(plan.converged);
    measured = plan.states.front();
  }
  EXPECT_NEAR(measured[0], 0.8, 0.01); EXPECT_NEAR(measured[1], 0, 0.01);
}
TEST(Mpc, InvalidInputAndIterationLimitAreReported) {
  MpcConfig config; config.iterations = 1; AxisMpc mpc(config);
  EXPECT_FALSE(mpc.solve({0, 0}, {}).converged);
  EXPECT_FALSE(mpc.solve({0, 0}, std::vector<Eigen::Vector2d>(config.horizon, {1, 0})).converged);
  EXPECT_FALSE(mpc.solve({NAN, 0}, std::vector<Eigen::Vector2d>(config.horizon, {1, 0})).converged);
  config.dt = 0; EXPECT_THROW(AxisMpc invalid(config), std::invalid_argument);
}

TEST(Ballistic, SolvesLowArcFromTheActualMuzzle) {
  const Eigen::Vector3d target(5.0, 1.0, 0.3);
  const Eigen::Vector3d offset(0.0741, 0.0, 0.0);
  const double speed = 22.0, gravity = 9.7833;
  const auto solution = ballistic_pointing_angles(target, offset, speed, gravity);
  const Eigen::Matrix3d rotation =
    (Eigen::AngleAxisd(solution.angles.x(), Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(-solution.angles.y(), Eigen::Vector3d::UnitY())).toRotationMatrix();
  const Eigen::Vector3d muzzle = rotation * offset;
  const double distance = (target.head<2>() - muzzle.head<2>()).norm();
  const double height = target.z() - muzzle.z();
  EXPECT_GT(solution.angles.y(), 0);
  EXPECT_LT(solution.angles.y(), pi / 4);  // Reject the high arc for this target.
  EXPECT_TRUE(std::isfinite(solution.flight_time));
  EXPECT_GT(solution.flight_time, 0);
  EXPECT_NEAR(height, distance * std::tan(solution.angles.y()) -
              gravity * distance * distance /
              (2 * speed * speed * std::cos(solution.angles.y()) *
               std::cos(solution.angles.y())), 1e-10);
  EXPECT_NEAR(solution.flight_time,
              distance / (speed * std::cos(solution.angles.y())), 1e-10);
}

TEST(Ballistic, SupportsLateralBarrelOffset) {
  const Eigen::Vector3d target(3.0, -0.8, -0.1);
  const Eigen::Vector3d offset(0.0741, -0.02, 0.01);
  const auto solution = ballistic_pointing_angles(
    target, offset, 24.0, 9.7833);
  const Eigen::Matrix3d rotation =
    (Eigen::AngleAxisd(solution.angles.x(), Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(-solution.angles.y(), Eigen::Vector3d::UnitY())).toRotationMatrix();
  const Eigen::Vector3d muzzle = rotation * offset;
  EXPECT_NEAR(std::atan2(target.y() - muzzle.y(), target.x() - muzzle.x()),
              solution.angles.x(), 1e-10);
}

TEST(Ballistic, RejectsInvalidAndUnreachableTargets) {
  EXPECT_THROW(ballistic_pointing_angles(
    {NAN, 0, 0}, {0, 0, 0}, 22, 9.8), std::invalid_argument);
  EXPECT_THROW(ballistic_pointing_angles(
    {1, 0, 0}, {0, 0, 0}, 0, 9.8), std::invalid_argument);
  EXPECT_THROW(ballistic_pointing_angles(
    {100, 0, 0}, {0, 0, 0}, 5, 9.8), std::runtime_error);
}
