#include "rm_aa_hardware/protocol.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>

using namespace rm_aa_hardware;
namespace {
Translator feedback() {
  Translator f{}; f.message.head = 0x71; f.message.tail = 0x4c;
  f.message.yaw = 0.2F; f.message.pitch = -0.1F; f.message.roll = 0.03F;
  f.message.status = 5; f.message.bias = 0.04F;
  return f;
}
Command target() { return {0.4, 0.1, 0.3, -0.2, 3.5, 1.25, -0.5, 12.0, 3, 1, true}; }
void expect_safe(const Translator & t) {
  const auto & m = t.message;
  EXPECT_EQ(m.head, 0x71); EXPECT_EQ(m.tail, 0x4c);
  EXPECT_EQ(m.armor_flag, 0); EXPECT_EQ(m.crc, 0); EXPECT_EQ(m.fire_allowance, 0);
  EXPECT_FLOAT_EQ(m.distance, 0); EXPECT_FLOAT_EQ(m.coo_x, 0); EXPECT_FLOAT_EQ(m.coo_y, 0);
  EXPECT_FLOAT_EQ(m.yaw_vel, 0); EXPECT_FLOAT_EQ(m.pitch_vel, 0);
  EXPECT_FLOAT_EQ(m.target_yaw, m.yaw); EXPECT_FLOAT_EQ(m.target_pitch, m.pitch);
}
}
TEST(HardwareProtocol, ExactWireBytesAndUnits) {
  ASSERT_TRUE(wire_platform_supported());
  const auto c = target();
  const auto out = output_frame(feedback().message, &c, true, true);
  const auto * bytes = reinterpret_cast<const unsigned char *>(out.data);
  ASSERT_EQ(sizeof(out), 64U);
  EXPECT_EQ(bytes[0], 0x71); EXPECT_EQ(bytes[13], 5); EXPECT_EQ(bytes[15], 3);
  EXPECT_EQ(bytes[44], 1); EXPECT_EQ(bytes[61], 1); EXPECT_EQ(bytes[62], 0); EXPECT_EQ(bytes[63], 0x4c);
  float value{}; std::memcpy(&value, out.data + 32, 4); EXPECT_FLOAT_EQ(value, 1250);
  std::memcpy(&value, out.data + 36, 4); EXPECT_FLOAT_EQ(value, -500);
  std::memcpy(&value, out.data + 45, 4); EXPECT_FLOAT_EQ(value, 0.4F);
  EXPECT_FLOAT_EQ(out.message.pitch_offset, 0); EXPECT_FLOAT_EQ(out.message.wheel_w, 0);
}
TEST(HardwareProtocol, SplitFramesNoiseAndConcatenation) {
  FrameParser parser;
  auto f = feedback();
  std::vector<char> stream{1, 2, 0x71, 3};
  stream.insert(stream.end(), f.data, f.data + 64);
  f.message.yaw = 0.8F;
  stream.insert(stream.end(), f.data, f.data + 64);
  EXPECT_TRUE(parser.append(stream.data(), 17).empty());
  const auto frames = parser.append(stream.data() + 17, stream.size() - 17);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_FLOAT_EQ(frames[0].message.yaw, 0.2F); EXPECT_FLOAT_EQ(frames[1].message.yaw, 0.8F);
}
TEST(HardwareProtocol, CorruptTailAndNanRecoverWithoutWaitingForAnotherFrame) {
  FrameParser parser;
  auto f = feedback(); f.message.tail = 0;
  EXPECT_TRUE(parser.append(f.data, 64).empty());
  f = feedback(); f.message.yaw = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(parser.append(f.data, 64).empty());
  f = feedback();
  const auto accepted = parser.append(f.data, 64);
  ASSERT_EQ(accepted.size(), 1U);
  EXPECT_FLOAT_EQ(accepted[0].message.roll, 0.03F);
}
TEST(HardwareProtocol, StaleMissingInvalidAndWrongModeHoldFeedback) {
  auto f = feedback(); auto c = target();
  expect_safe(output_frame(f.message, &c, false, true));
  expect_safe(output_frame(f.message, nullptr, true, true));
  f.message.status = 1;
  expect_safe(output_frame(f.message, &c, true, true));
  f = feedback(); c.yaw = std::numeric_limits<double>::quiet_NaN();
  expect_safe(output_frame(f.message, &c, true, true));
  c = target(); c.center_x_m = std::numeric_limits<double>::max();
  expect_safe(output_frame(f.message, &c, true, true));
  c = target(); c.target_id = 0;
  expect_safe(output_frame(f.message, &c, true, true));
}
TEST(HardwareProtocol, FireRequiresBothExplicitEnableAndStableState) {
  auto c = target(); auto f = feedback();
  EXPECT_EQ(output_frame(f.message, &c, true, false).message.fire_allowance, 0);
  c.tracking_state = 2;
  const auto unstable = output_frame(f.message, &c, true, true);
  EXPECT_EQ(unstable.message.crc, 2); EXPECT_EQ(unstable.message.fire_allowance, 0);
  c.tracking_state = 1; c.fire = false;
  EXPECT_EQ(output_frame(f.message, &c, true, true).message.fire_allowance, 0);
}
TEST(HardwareFeedback, VelocityWrapAndGap) {
  AttitudeEstimator estimator; auto f = feedback();
  f.message.yaw = 3.13F;
  EXPECT_FALSE(estimator.update(f.message, 1.0, 0.15).velocity_valid);
  f.message.yaw = -3.13F;
  const auto second = estimator.update(f.message, 1.02, 0.15);
  ASSERT_TRUE(second.velocity_valid);
  EXPECT_NEAR(second.velocity[0], (6.283185307179586 - 6.26) / 0.02, 1e-4);
  EXPECT_FALSE(estimator.update(f.message, 2.0, 0.15).velocity_valid);
  EXPECT_FALSE(estimator.update(f.message, 2.0, 0.15).velocity_valid);
  estimator.reset();
  EXPECT_FALSE(estimator.update(f.message, 3.0, 0.15).velocity_valid);
}
TEST(HardwareFeedback, PositivePitchRaisesForwardAxisAndRollIsPreserved) {
  const auto q = attitude_quaternion(0, 0.3, 0);
  const double forward_z = 2 * (q[0]*q[2] - q[3]*q[1]);
  EXPECT_NEAR(forward_z, std::sin(0.3), 1e-12);
  const auto roll = attitude_quaternion(0, 0, 0.2);
  EXPECT_NEAR(roll[0], std::sin(0.1), 1e-12);
  const auto all = attitude_quaternion(0.7, -0.3, 0.2);
  double norm = 0; for (double value : all) norm += value*value;
  EXPECT_NEAR(norm, 1.0, 1e-12);
}
TEST(HardwareTrajectory, SamplesByTimeAndExpiresAtEnd) {
  const std::vector<TrajectoryPoint> points{{0.1,{1,2},{3,4}}, {0.2,{2,3},{5,6}}};
  Command c;
  ASSERT_TRUE(sample_trajectory(points, 0.05, {0,0}, {0,0}, c));
  EXPECT_DOUBLE_EQ(c.yaw, 0.5); EXPECT_DOUBLE_EQ(c.pitch, 1); EXPECT_DOUBLE_EQ(c.yaw_velocity, 1.5);
  ASSERT_TRUE(sample_trajectory(points, 0.15, {0,0}, {0,0}, c));
  EXPECT_NEAR(c.yaw, 1.5, 1e-12); EXPECT_NEAR(c.pitch_velocity, 5, 1e-12);
  EXPECT_FALSE(sample_trajectory(points, 0.21, {0,0}, {0,0}, c));
  EXPECT_FALSE(sample_trajectory(points, -0.1, {0,0}, {0,0}, c));
}
TEST(HardwareTrajectory, RejectsBadTimesAndNonfiniteValues) {
  EXPECT_FALSE(valid_trajectory({}));
  EXPECT_FALSE(valid_trajectory({{0.1,{},{}}, {0.1,{},{}}}));
  EXPECT_FALSE(valid_trajectory({{-0.1,{},{}}}));
  EXPECT_FALSE(valid_trajectory({{0.1,{std::numeric_limits<double>::infinity(),0},{}}}));
}
