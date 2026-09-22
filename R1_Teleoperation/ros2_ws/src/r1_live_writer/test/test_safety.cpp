#include <array>
#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "r1_live_writer/safety.hpp"
#include "r1_live_writer/transport.hpp"

namespace r1_live_writer
{
namespace
{

LiveEnvironment complete_environment()
{
  LiveEnvironment environment;
  environment.dry_run_disabled = true;
  environment.actuation_enabled = true;
  environment.off_charger_confirmed = true;
  environment.clear_area_confirmed = true;
  environment.estop_ready_confirmed = true;
  environment.commissioning_confirmed = true;
  environment.commissioning_token = "commissioning-token";
  environment.vr_source_ip = "192.168.8.42";
  return environment;
}

AuthorizationInput complete_input()
{
  AuthorizationInput input;
  input.send_commands = true;
  input.feature_enabled = true;
  input.commissioning_parameter_confirmed = true;
  input.commissioning_token_matches = true;
  input.vr_source_matches = true;
  input.profile_is_slow_safe = true;
  input.kill_clear_fresh = true;
  input.deadman_active_fresh = true;
  input.state_fresh = true;
  input.motor_health_required = true;
  input.motors_healthy_fresh = true;
  input.command_fresh = true;
  input.prepared = true;
  input.require_prepared = true;
  input.environment = complete_environment();
  return input;
}

TEST(LiveGate, CompleteGateAllowsSend)
{
  const GateDecision decision = authorize_live_send(complete_input());
  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(decision.reason, "authorized");
}

TEST(LiveGate, SendCommandsFalseAlwaysBlocks)
{
  auto input = complete_input();
  input.send_commands = false;
  const GateDecision decision = authorize_live_send(input);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.reason, "send_commands=false");
}

TEST(LiveGate, KillDeadmanWatchdogAndStateFailClosed)
{
  auto input = complete_input();
  input.kill_clear_fresh = false;
  EXPECT_FALSE(authorize_live_send(input).allowed);
  input.kill_clear_fresh = true;
  input.deadman_active_fresh = false;
  EXPECT_FALSE(authorize_live_send(input).allowed);
  input.deadman_active_fresh = true;
  input.state_fresh = false;
  EXPECT_FALSE(authorize_live_send(input).allowed);
  input.state_fresh = true;
  input.command_fresh = false;
  EXPECT_FALSE(authorize_live_send(input).allowed);
}

TEST(LiveGate, PhysicalMotorHealthMustBeFreshAndTrue)
{
  auto input = complete_input();
  input.motors_healthy_fresh = false;
  const GateDecision decision = authorize_live_send(input);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.reason, "motor_health_missing_stale_or_unhealthy");
}

TEST(LiveGate, MockModeDoesNotRequirePhysicalMotorHealth)
{
  auto input = complete_input();
  input.motor_health_required = false;
  input.motors_healthy_fresh = false;
  const GateDecision decision = authorize_live_send(input);
  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(decision.reason, "authorized");
}

TEST(LiveGate, MissingEnvironmentFlagBlocks)
{
  auto input = complete_input();
  input.environment.off_charger_confirmed = false;
  const GateDecision decision = authorize_live_send(input);
  EXPECT_FALSE(decision.allowed);
  EXPECT_NE(decision.reason.find("ROBOT_CONFIRM_OFF_CHARGER=1"), std::string::npos);
}

TEST(LiveGate, OnlySlowSafeCanReachFirstLiveSend)
{
  auto input = complete_input();
  input.profile_is_slow_safe = false;
  EXPECT_FALSE(authorize_live_send(input).allowed);
}

TEST(NetworkValidation, AcceptsOnlyRfc1918UnicastAddresses)
{
  EXPECT_TRUE(valid_private_unicast_ipv4("10.0.0.1"));
  EXPECT_TRUE(valid_private_unicast_ipv4("10.255.255.254"));
  EXPECT_TRUE(valid_private_unicast_ipv4("172.16.0.1"));
  EXPECT_TRUE(valid_private_unicast_ipv4("172.31.255.254"));
  EXPECT_TRUE(valid_private_unicast_ipv4("192.168.0.1"));
  EXPECT_TRUE(valid_private_unicast_ipv4("192.168.123.164"));

  EXPECT_FALSE(valid_private_unicast_ipv4("8.8.8.8"));
  EXPECT_FALSE(valid_private_unicast_ipv4("172.15.255.254"));
  EXPECT_FALSE(valid_private_unicast_ipv4("172.32.0.1"));
  EXPECT_FALSE(valid_private_unicast_ipv4("169.254.1.1"));
  EXPECT_FALSE(valid_private_unicast_ipv4("127.0.0.1"));
  EXPECT_FALSE(valid_private_unicast_ipv4("224.0.0.1"));
  EXPECT_FALSE(valid_private_unicast_ipv4("239.255.255.250"));
  EXPECT_FALSE(valid_private_unicast_ipv4("0.0.0.0"));
  EXPECT_FALSE(valid_private_unicast_ipv4("255.255.255.255"));
  EXPECT_FALSE(valid_private_unicast_ipv4(""));
  EXPECT_FALSE(valid_private_unicast_ipv4("not-an-ip"));
  EXPECT_FALSE(valid_private_unicast_ipv4("192.168.1"));
  EXPECT_FALSE(valid_private_unicast_ipv4("192.168.1.256"));
}

TEST(Limits, LocomotionClampsMagnitudeAndRate)
{
  const auto result = clamp_velocity(
    {9.0, -9.0, 9.0}, {0.0, 0.0, 0.0}, 1.0, VelocityLimits{});
  EXPECT_DOUBLE_EQ(result[0], 0.025);
  EXPECT_DOUBLE_EQ(result[1], -0.025);
  EXPECT_DOUBLE_EQ(result[2], 0.05);
}

TEST(Limits, HeadClampsMagnitudeAndRate)
{
  const auto result = clamp_head(
    {9.0, -9.0}, {0.0, 0.0}, 1.0, HeadLimits{});
  EXPECT_DOUBLE_EQ(result[0], 0.035);
  EXPECT_DOUBLE_EQ(result[1], -0.025);
}

TEST(HeadsetRelativeHead, ZeroDeltaPreservesMeasuredNonzeroOrigin)
{
  const std::array<double, 2> measured_origin{1.4299, 0.0073};
  const auto relative_delta = clamp_head(
    {0.0, 0.0}, {0.0, 0.0}, 0.01, HeadLimits{});
  const auto target = absolute_head_target(
    measured_origin, relative_delta, HeadAbsoluteLimits{});

  EXPECT_EQ(relative_delta, (std::array<double, 2>{0.0, 0.0}));
  EXPECT_EQ(target, measured_origin);
  EXPECT_TRUE(head_within_absolute_limits(target, HeadAbsoluteLimits{}));
}

TEST(HeadsetRelativeHead, FirstDeltaIsRateLimitedBeforeOriginAddition)
{
  const std::array<double, 2> measured_origin{1.4299, 0.0073};
  const auto relative_delta = clamp_head(
    {0.30, -0.20}, {0.0, 0.0}, 1.0, HeadLimits{});
  const auto target = absolute_head_target(
    measured_origin, relative_delta, HeadAbsoluteLimits{});

  ASSERT_DOUBLE_EQ(relative_delta[0], 0.035);
  ASSERT_DOUBLE_EQ(relative_delta[1], -0.025);
  EXPECT_NEAR(target[0], 1.4649, 1e-12);
  EXPECT_NEAR(target[1], -0.0177, 1e-12);
}

TEST(HeadsetRelativeHead, AbsoluteTargetClampsToOfficialR1Envelope)
{
  const HeadAbsoluteLimits official_limits{};
  const auto positive = absolute_head_target(
    {1.95, 0.60}, {0.35, 0.25}, official_limits);
  const auto negative = absolute_head_target(
    {-1.95, -0.60}, {-0.35, -0.25}, official_limits);

  EXPECT_DOUBLE_EQ(positive[0], official_limits.yaw_max);
  EXPECT_DOUBLE_EQ(positive[1], official_limits.pitch_max);
  EXPECT_DOUBLE_EQ(negative[0], official_limits.yaw_min);
  EXPECT_DOUBLE_EQ(negative[1], official_limits.pitch_min);
  EXPECT_TRUE(head_within_absolute_limits(positive, official_limits));
  EXPECT_TRUE(head_within_absolute_limits(negative, official_limits));
}

TEST(HeadsetRelativeHead, NonfiniteOrOutOfEnvelopeSeedFailsClosed)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const HeadAbsoluteLimits official_limits{};

  EXPECT_FALSE(head_within_absolute_limits({nan, 0.0}, official_limits));
  EXPECT_FALSE(head_within_absolute_limits({0.0, nan}, official_limits));
  EXPECT_FALSE(head_within_absolute_limits({2.0072, 0.0}, official_limits));

  const auto nonfinite_delta = absolute_head_target(
    {1.4299, 0.0073}, {nan, 0.0}, official_limits);
  const auto outside_seed = absolute_head_target(
    {2.0072, 0.0}, {0.0, 0.0}, official_limits);
  EXPECT_FALSE(finite_values(nonfinite_delta));
  EXPECT_FALSE(finite_values(outside_seed));
}

TEST(HeadsetRelativeHead, InvalidAbsoluteEnvelopeFailsClosed)
{
  HeadAbsoluteLimits invalid_limits{};
  invalid_limits.yaw_min = invalid_limits.yaw_max + 0.01;

  EXPECT_FALSE(head_within_absolute_limits({0.0, 0.0}, invalid_limits));
  EXPECT_FALSE(finite_values(absolute_head_target(
    {0.0, 0.0}, {0.0, 0.0}, invalid_limits)));
}

TEST(HeadsetRelativeHead, StaleStateOrCommandIsRejectedBeforeShaping)
{
  auto input = complete_input();
  input.state_fresh = false;
  GateDecision decision = authorize_live_send(input);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.reason, "joint_state_missing_stale_or_invalid");

  input.state_fresh = true;
  input.command_fresh = false;
  decision = authorize_live_send(input);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.reason, "command_missing_stale_or_invalid");
}

TEST(HeadsetRelativeHead, AbsoluteSaturationDoesNotWindUpRelativeRateLimiter)
{
  const std::array<double, 2> origin{1.95, 0.0};
  const HeadLimits relative_limits{};
  const HeadAbsoluteLimits absolute_limits{};

  const auto first_outward_delta = clamp_head(
    {0.35, 0.0}, {0.0, 0.0}, 1.0, relative_limits);
  const auto first_target = absolute_head_target(
    origin, first_outward_delta, absolute_limits);
  const std::array<double, 2> first_effective_delta{
    first_target[0] - origin[0], first_target[1] - origin[1]};
  const auto outward_delta = clamp_head(
    {0.35, 0.0}, first_effective_delta, 1.0, relative_limits);
  const auto saturated_target = absolute_head_target(
    origin, outward_delta, absolute_limits);
  const std::array<double, 2> effective_delta{
    saturated_target[0] - origin[0], saturated_target[1] - origin[1]};
  ASSERT_NEAR(effective_delta[0], 0.0571, 1e-12);

  const auto inward_delta = clamp_head(
    {-0.35, 0.0}, effective_delta, 1.0, relative_limits);
  const auto inward_target = absolute_head_target(
    origin, inward_delta, absolute_limits);
  EXPECT_LT(inward_target[0], saturated_target[0]);
  EXPECT_NEAR(inward_target[0], 1.9721, 1e-12);
}

TEST(HeadRecenter, StepIsMonotonicRateLimitedAndCannotOvershoot)
{
  const auto positive = step_head_toward_zero({1.4299, 0.0073}, 0.02, 0.08, 0.05);
  EXPECT_NEAR(positive[0], 1.4283, 1e-12);
  EXPECT_NEAR(positive[1], 0.0063, 1e-12);

  const auto negative = step_head_toward_zero({-1.4299, -0.0004}, 0.02, 0.08, 0.05);
  EXPECT_NEAR(negative[0], -1.4283, 1e-12);
  EXPECT_DOUBLE_EQ(negative[1], 0.0);
  EXPECT_LT(std::abs(positive[0]), 1.4299);
  EXPECT_LT(std::abs(negative[0]), 1.4299);

  const auto capped_dt = step_head_toward_zero({1.0, -1.0}, 9.0, 0.08, 0.05);
  EXPECT_NEAR(capped_dt[0], 0.992, 1e-12);
  EXPECT_NEAR(capped_dt[1], -0.995, 1e-12);
}

TEST(HeadRecenter, InvalidInputFailsClosed)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(finite_values(step_head_toward_zero({nan, 0.0}, 0.02, 0.08, 0.05)));
  EXPECT_FALSE(finite_values(step_head_toward_zero({0.0, 0.0}, nan, 0.08, 0.05)));
  EXPECT_FALSE(finite_values(step_head_toward_zero({0.0, 0.0}, 0.02, 0.0, 0.05)));
}

TEST(PrepareRearmGate, FullLocomotionSequenceRequiresReleaseNeutralAndNewPress)
{
  PrepareRearmGate gate;
  EXPECT_EQ(gate.state(), PrepareRearmState::Unprepared);
  EXPECT_FALSE(gate.ready());

  gate.prepared(true);
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitDeadmanRelease);
  EXPECT_FALSE(gate.observe_deadman(true));
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitDeadmanRelease);

  EXPECT_FALSE(gate.observe_deadman(false));
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitNeutralVelocity);
  EXPECT_TRUE(gate.observe_velocity({0.0, 0.0, 0.0}));
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitDeadmanPress);

  EXPECT_TRUE(gate.observe_deadman(true));
  EXPECT_EQ(gate.state(), PrepareRearmState::Ready);
  EXPECT_TRUE(gate.ready());
}

TEST(PrepareRearmGate, EarlyPressBeforeNeutralCannotArm)
{
  PrepareRearmGate gate;
  gate.prepared(true);
  EXPECT_FALSE(gate.observe_deadman(false));
  ASSERT_EQ(gate.state(), PrepareRearmState::AwaitNeutralVelocity);

  EXPECT_FALSE(gate.observe_deadman(true));
  EXPECT_FALSE(gate.ready());
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitNeutralVelocity);
  EXPECT_FALSE(gate.observe_velocity({0.0, 0.0, 0.0}));
  EXPECT_FALSE(gate.ready());

  // A second release is required after the premature press before a neutral
  // processed velocity sample can advance the state machine.
  EXPECT_FALSE(gate.observe_deadman(false));
  EXPECT_TRUE(gate.observe_velocity({0.0, 0.0, 0.0}));
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitDeadmanPress);
}

TEST(PrepareRearmGate, NonNeutralVelocityBlocksLocomotionRearm)
{
  PrepareRearmGate gate;
  gate.prepared(true);
  EXPECT_FALSE(gate.observe_deadman(false));

  EXPECT_FALSE(gate.observe_velocity({0.0, 0.0101, 0.0}));
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitNeutralVelocity);
  EXPECT_FALSE(gate.ready());

  EXPECT_TRUE(gate.observe_velocity({0.01, -0.01, 0.01}));
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitDeadmanPress);
  EXPECT_FALSE(gate.ready());
}

TEST(PrepareRearmGate, HeadOnlySkipsNeutralVelocityStep)
{
  PrepareRearmGate gate;
  gate.prepared(false);
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitDeadmanRelease);

  EXPECT_FALSE(gate.observe_deadman(false));
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitDeadmanPress);
  EXPECT_TRUE(gate.observe_deadman(true));
  EXPECT_TRUE(gate.ready());
}

TEST(PrepareRearmGate, ReleaseAfterReadyDisarmsAgain)
{
  PrepareRearmGate gate;
  gate.prepared(true);
  EXPECT_FALSE(gate.observe_deadman(false));
  EXPECT_TRUE(gate.observe_velocity({0.0, 0.0, 0.0}));
  EXPECT_TRUE(gate.observe_deadman(true));
  ASSERT_TRUE(gate.ready());

  EXPECT_FALSE(gate.observe_deadman(false));
  EXPECT_FALSE(gate.ready());
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitNeutralVelocity);
}

TEST(Limits, CorruptArmSdkFeedbackIsNeverEchoedAsASeed)
{
  ArmSdkPositions positions{};
  EXPECT_TRUE(sane_arm_sdk_feedback(positions));
  positions[4] = 100.0;
  EXPECT_FALSE(sane_arm_sdk_feedback(positions));
  positions[4] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sane_arm_sdk_feedback(positions));
}

TEST(MockTransport, HeadMustSeedAllFieldsBeforeCommand)
{
  MockTransport transport;
  ASSERT_TRUE(transport.initialize("mock0", true, true).ok);
  ArmSdkPositions positions{};
  EXPECT_FALSE(transport.command_head(positions).ok);
  EXPECT_TRUE(transport.seed_head(positions).ok);
  EXPECT_TRUE(transport.command_head(positions).ok);
  EXPECT_TRUE(transport.hold_head(positions).ok);
  EXPECT_TRUE(transport.release_head(positions).ok);
  EXPECT_FALSE(transport.command_head(positions).ok);
  EXPECT_EQ(transport.head_seed_calls(), 1U);
  EXPECT_EQ(transport.head_command_calls(), 1U);
  EXPECT_EQ(transport.head_hold_calls(), 1U);
  EXPECT_EQ(transport.head_release_calls(), 1U);
}

TEST(MockTransport, WeightedHeadSeedAndCommandsRequireFiniteUnitIntervalWeight)
{
  MockTransport transport;
  ASSERT_TRUE(transport.initialize("mock0", true, false).ok);
  ArmSdkPositions positions{};
  positions[0] = 0.125;
  positions[12] = -0.25;

  EXPECT_FALSE(transport.seed_head_weighted(positions, -0.001).ok);
  EXPECT_FALSE(transport.seed_head_weighted(positions, 1.001).ok);
  EXPECT_FALSE(
    transport.seed_head_weighted(
      positions, std::numeric_limits<double>::quiet_NaN()).ok);
  EXPECT_EQ(transport.head_seed_calls(), 0U);

  ASSERT_TRUE(transport.seed_head_weighted(positions, 0.0).ok);
  EXPECT_EQ(transport.last_head(), positions);
  EXPECT_DOUBLE_EQ(transport.last_head_weight(), 0.0);

  positions[11] = 0.05;
  ASSERT_TRUE(transport.command_head_weighted(positions, 0.125).ok);
  EXPECT_EQ(transport.last_head(), positions);
  EXPECT_DOUBLE_EQ(transport.last_head_weight(), 0.125);
  positions[12] = -0.2;
  ASSERT_TRUE(transport.hold_head(positions).ok);
  EXPECT_EQ(transport.last_head(), positions);
  EXPECT_DOUBLE_EQ(transport.last_head_weight(), 0.125);
  EXPECT_FALSE(transport.command_head_weighted(positions, -0.1).ok);
  EXPECT_FALSE(transport.command_head_weighted(positions, 1.1).ok);
  EXPECT_FALSE(
    transport.command_head_weighted(
      positions, std::numeric_limits<double>::infinity()).ok);
  EXPECT_EQ(transport.head_command_calls(), 1U);
  EXPECT_EQ(transport.head_hold_calls(), 1U);
  EXPECT_DOUBLE_EQ(transport.last_head_weight(), 0.125);

  ASSERT_TRUE(transport.release_head(positions).ok);
  EXPECT_DOUBLE_EQ(transport.last_head_weight(), 0.0);
}

TEST(MockTransport, LegacyHeadMethodsRetainFullWeightSemantics)
{
  MockTransport transport;
  ASSERT_TRUE(transport.initialize("mock0", true, false).ok);
  ArmSdkPositions positions{};

  ASSERT_TRUE(transport.seed_head(positions).ok);
  EXPECT_DOUBLE_EQ(transport.last_head_weight(), 1.0);
  ASSERT_TRUE(transport.command_head(positions).ok);
  EXPECT_DOUBLE_EQ(transport.last_head_weight(), 1.0);
  ASSERT_TRUE(transport.hold_head(positions).ok);
  EXPECT_DOUBLE_EQ(transport.last_head_weight(), 1.0);
}

TEST(ArmReleaseWeight, ScheduleIsMonotonicAndNeverExceedsItsStartingWeight)
{
  constexpr std::size_t steps = 50;
  for (const double start_weight : {0.0, 0.01, 0.125, 1.0}) {
    double previous = start_weight;
    for (std::size_t step = 1; step <= steps; ++step) {
      const double weight = detail::descending_arm_release_weight(
        start_weight, step, steps);
      ASSERT_TRUE(std::isfinite(weight));
      EXPECT_GE(weight, 0.0);
      EXPECT_LE(weight, start_weight);
      EXPECT_LE(weight, previous);
      previous = weight;
    }
    EXPECT_DOUBLE_EQ(previous, 0.0);
  }
}

TEST(ArmReleaseWeight, InvalidScheduleInputsFailClosed)
{
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(-0.1, 1, 50)));
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(1.1, 1, 50)));
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(
    std::numeric_limits<double>::quiet_NaN(), 1, 50)));
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(0.5, 1, 0)));
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(0.5, 51, 50)));
}

TEST(MockTransport, StopAndPrepareAreObservableWithoutRobot)
{
  MockTransport transport;
  ASSERT_TRUE(transport.initialize("mock0", false, true).ok);
  EXPECT_TRUE(transport.prepare().ok);
  EXPECT_TRUE(transport.set_velocity(0.1, 0.0, 0.0, 0.12).ok);
  EXPECT_TRUE(transport.stop_locomotion().ok);
  EXPECT_EQ(transport.prepare_calls(), 1U);
  EXPECT_EQ(transport.velocity_calls(), 1U);
  EXPECT_EQ(transport.stop_calls(), 1U);
  EXPECT_EQ(transport.last_velocity(), (std::array<double, 3>{0.0, 0.0, 0.0}));
}

TEST(MockTransport, CancelledPrepareDoesNotRecordStandUp)
{
  MockTransport transport;
  ASSERT_TRUE(transport.initialize("mock0", false, true).ok);
  const TransportResult result = transport.prepare([]() {return true;});
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.detail.find("cancelled"), std::string::npos);
  EXPECT_EQ(transport.prepare_calls(), 0U);
}

}  // namespace
}  // namespace r1_live_writer
