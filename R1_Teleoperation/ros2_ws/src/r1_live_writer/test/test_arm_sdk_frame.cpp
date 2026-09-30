#include "r1_live_writer/arm_sdk_frame.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

namespace r1_live_writer
{
namespace
{
using Frame = unitree_hg::msg::dds_::LowCmd_;

ArmSdkPositions distinctive_positions()
{
  ArmSdkPositions positions{};
  for (std::size_t i = 0; i < positions.size(); ++i) {
    positions[i] = (static_cast<double>(i) - 6.0) * 0.125;
  }
  return positions;
}

TEST(ArmSdkFrame, MapsMeasuredMetadataAndTargetsWithoutHoming)
{
  const auto positions = distinctive_positions();
  Frame frame;
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(positions, 1.0F, 231, -0.375F, frame));
  EXPECT_EQ(frame.mode_pr(), 100);
  EXPECT_EQ(frame.mode_machine(), 231);
  EXPECT_FLOAT_EQ(frame.motor_cmd()[12].q(), -0.375F);
  constexpr std::array<std::size_t, 13> indices = {
    15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 13, 29, 30};
  for (std::size_t i = 0; i < indices.size(); ++i) {
    EXPECT_FLOAT_EQ(frame.motor_cmd()[indices[i]].q(), positions[i]) << indices[i];
  }
}

TEST(ArmSdkFrame, OnlyUpstreamUpperBodySubsetIsActiveWithExactGains)
{
  Frame frame;
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(distinctive_positions(), 1.0F, 7, -0.375F, frame));
  constexpr std::array<float, 35> kp = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 50, 50, 0,
    50, 50, 40, 40, 30, 0, 0, 50, 50, 40, 40, 30, 0, 0, 15, 15, 0, 0, 0, 0};
  constexpr std::array<float, 35> kd = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 0,
    2, 2, 2, 2, 2, 0, 0, 2, 2, 2, 2, 2, 0, 0, 1, 1, 0, 0, 0, 0};
  for (std::size_t i = 0; i < frame.motor_cmd().size(); ++i) {
    const auto & motor = frame.motor_cmd()[i];
    EXPECT_EQ(motor.mode(), kp[i] != 0 ? 1 : 0) << i;
    EXPECT_FLOAT_EQ(motor.kp(), kp[i]) << i;
    EXPECT_FLOAT_EQ(motor.kd(), kd[i]) << i;
    EXPECT_FLOAT_EQ(motor.dq(), 0) << i;
    EXPECT_FLOAT_EQ(motor.tau(), 0) << i;
    EXPECT_EQ(motor.reserve(), 0U) << i;
    if (kp[i] == 0) {
      EXPECT_FLOAT_EQ(motor.q(), 0) << i;
    }
  }
  EXPECT_EQ(frame.reserve(), (std::array<std::uint32_t, 4>{}));
}

// Constants below were independently generated with the installed SDK2 Python
// packing format '<2B2x' + 'B3x5fI'*35 + '5I', struct.pack, and vendor
// crc_amd64.so::crc32_core. They are not calculated by the code under test.
TEST(ArmSdkFrame, MatchesIndependentVendorGoldenChecksums)
{
  EXPECT_EQ(hg_low_cmd_crc(Frame{}), 0xfe172f9fU);
  Frame frame;
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(distinctive_positions(), 1.0F, 7, -0.375F, frame));
  EXPECT_EQ(frame.crc(), 0x71f142bfU);
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(distinctive_positions(), 0.0F, 7, -0.375F, frame));
  EXPECT_EQ(frame.crc(), 0x931f299aU);
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(distinctive_positions(), 1.0F, 231, -0.375F, frame));
  EXPECT_EQ(frame.crc(), 0x758c1a6aU);
}

TEST(ArmSdkFrame, ChecksumCoversEveryFieldInLastMotorAndAllReserves)
{
  Frame frame;
  frame.mode_pr(0x12);
  frame.mode_machine(0x34);
  auto & motor = frame.motor_cmd()[34];
  motor.mode(0x56);
  motor.q(1.25F);
  motor.dq(-2.5F);
  motor.tau(3.75F);
  motor.kp(4.5F);
  motor.kd(-5.25F);
  motor.reserve(0x89abcdefU);
  frame.reserve({0x01234567U, 0x89abcdefU, 0xfedcba98U, 0x76543210U});
  frame.crc(0xffffffffU);
  EXPECT_EQ(hg_low_cmd_crc(frame), 0x15e135cdU);
  frame.crc(0);
  EXPECT_EQ(hg_low_cmd_crc(frame), 0x15e135cdU);  // crc itself is excluded.
}

TEST(ArmSdkFrame, ReleaseRetainsTargetsAndGainsWhileResealingWeight)
{
  Frame active;
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(distinctive_positions(), 1.0F, 7, -0.375F, active));
  Frame released;
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(distinctive_positions(), 0.0F, 7, -0.375F, released));
  EXPECT_EQ(released.mode_pr(), 0);
  EXPECT_EQ(released.mode_machine(), active.mode_machine());
  EXPECT_EQ(released.motor_cmd(), active.motor_cmd());
  EXPECT_EQ(released.crc(), hg_low_cmd_crc(released));
  EXPECT_NE(released.crc(), active.crc());
}

TEST(ArmSdkFrame, RebuildClearsPreviouslyPopulatedLowerBodyAndReservedFields)
{
  Frame frame;
  for (auto & motor : frame.motor_cmd()) {
    motor.mode(255);
    motor.q(42);
    motor.dq(42);
    motor.tau(42);
    motor.kp(42);
    motor.kd(42);
    motor.reserve(0xffffffffU);
  }
  frame.reserve({1, 2, 3, 4});
  frame.crc(123);
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(distinctive_positions(), 1.0F, 7, -0.375F, frame));
  EXPECT_EQ(frame.crc(), 0x71f142bfU);
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_EQ(frame.motor_cmd()[i], unitree_hg::msg::dds_::MotorCmd_{});
  }
  EXPECT_EQ(frame.reserve(), (std::array<std::uint32_t, 4>{}));
}

TEST(ArmSdkFrame, WeightUsesBoundedPercentageTruncation)
{
  Frame frame;
  for (const auto & item : std::array<std::pair<float, int>, 6>{
      {{0.0F, 0}, {0.009F, 0}, {0.019F, 1}, {0.125F, 12}, {0.999F, 99}, {1.0F, 100}}})
  {
    ASSERT_TRUE(build_r1_a5_arm_sdk_frame({}, item.first, 0, 0, frame));
    EXPECT_EQ(frame.mode_pr(), item.second);
    EXPECT_EQ(frame.crc(), hg_low_cmd_crc(frame));
  }
}

TEST(ArmSdkFrame, InvalidWeightsAndWaistLeaveOutputUnmodified)
{
  Frame frame;
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(distinctive_positions(), 0.5F, 7, -0.375F, frame));
  const auto before = frame;
  for (float bad_weight : {
      -0.01F, 1.01F, std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()})
  {
    EXPECT_FALSE(build_r1_a5_arm_sdk_frame({}, bad_weight, 1, 0, frame));
    EXPECT_EQ(frame, before);
  }
  for (float bad_waist : {
      std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
      std::numeric_limits<float>::quiet_NaN()})
  {
    EXPECT_FALSE(build_r1_a5_arm_sdk_frame({}, 1, 1, bad_waist, frame));
    EXPECT_EQ(frame, before);
  }
}

TEST(ArmSdkFrame, EveryInvalidOrOverflowingTargetLeavesOutputUnmodified)
{
  Frame frame;
  ASSERT_TRUE(build_r1_a5_arm_sdk_frame(distinctive_positions(), 0.5F, 7, -0.375F, frame));
  const auto before = frame;
  for (std::size_t i = 0; i < 13; ++i) {
    for (double invalid : {
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max(),
        -std::numeric_limits<double>::max()})
    {
      auto positions = distinctive_positions();
      positions[i] = invalid;
      EXPECT_FALSE(build_r1_a5_arm_sdk_frame(positions, 1, 1, 0, frame)) << i;
      EXPECT_EQ(frame, before) << i;
    }
  }
}

TEST(ArmSdkFrame, FiniteFloatExtremaAndMetadataByteBoundariesArePreserved)
{
  // Numeric encoding boundaries, not physical commands: joint limits are the
  // writer's responsibility and no DDS publisher is constructed in these tests.
  ArmSdkPositions positions{};
  positions[0] = std::numeric_limits<float>::max();
  positions[12] = -std::numeric_limits<float>::max();
  Frame frame;
  for (std::uint8_t mode : {std::uint8_t{0}, std::uint8_t{255}}) {
    ASSERT_TRUE(build_r1_a5_arm_sdk_frame(positions, 1, mode, 0, frame));
    EXPECT_EQ(frame.mode_machine(), mode);
    EXPECT_FLOAT_EQ(frame.motor_cmd()[15].q(), std::numeric_limits<float>::max());
    EXPECT_FLOAT_EQ(frame.motor_cmd()[30].q(), -std::numeric_limits<float>::max());
  }
}

}  // namespace
}  // namespace r1_live_writer
