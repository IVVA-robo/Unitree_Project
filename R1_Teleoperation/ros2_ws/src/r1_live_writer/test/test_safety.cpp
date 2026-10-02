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

TEST(ArmSdkMetadataGate, RequiresFreshMeasuredSeedAndKeepsRollTargetFixed)
{
  detail::ArmSdkMetadataGate gate;
  const auto now = detail::ArmSdkMetadataGate::Clock::now();
  EXPECT_FALSE(gate.seed(now));
  EXPECT_FALSE(gate.permits(0.0, now));
  gate.observe(7, 0.125, now);
  ASSERT_TRUE(gate.seed(now));
  EXPECT_EQ(gate.mode_machine(), 7);
  EXPECT_FLOAT_EQ(gate.waist_roll(), 0.125F);
  gate.observe(7, 0.2, now + std::chrono::milliseconds(10));
  EXPECT_TRUE(gate.permits(1.0, now + std::chrono::milliseconds(20)));
  EXPECT_FLOAT_EQ(gate.waist_roll(), 0.125F);
}

TEST(ArmSdkMetadataGate, StaleOrChangedModeBlocksPositiveButAllowsRelease)
{
  detail::ArmSdkMetadataGate gate;
  const auto now = detail::ArmSdkMetadataGate::Clock::now();
  gate.observe(1, 0.01, now);
  ASSERT_TRUE(gate.seed(now));
  EXPECT_TRUE(gate.permits(1.0, now + std::chrono::milliseconds(500)));
  EXPECT_FALSE(gate.permits(1.0, now + std::chrono::milliseconds(501)));
  EXPECT_TRUE(gate.permits(0.0, now + std::chrono::seconds(5)));
  EXPECT_FALSE(gate.seed(now + std::chrono::seconds(5)));
  gate.observe(2, 0.02, now + std::chrono::seconds(6));
  EXPECT_FALSE(gate.permits(0.01, now + std::chrono::seconds(6)));
  EXPECT_TRUE(gate.permits(0.0, now + std::chrono::seconds(6)));
  EXPECT_EQ(gate.mode_machine(), 1);
}

TEST(ArmSdkMetadataGate, InvalidSamplesWeightsAndBackwardTimeDoNotAuthorize)
{
  detail::ArmSdkMetadataGate gate;
  const auto now = detail::ArmSdkMetadataGate::Clock::now();
  gate.observe(0, std::numeric_limits<double>::quiet_NaN(), now);
  EXPECT_FALSE(gate.seed(now));
  gate.observe(0, 1.1, now);
  EXPECT_FALSE(gate.seed(now));
  gate.observe(0, 0.0, now);
  EXPECT_FALSE(gate.seed(now - std::chrono::milliseconds(1)));
  ASSERT_TRUE(gate.seed(now));
  EXPECT_TRUE(gate.permits(1.0, now));  // mode zero is measured, not guessed.
  EXPECT_FALSE(gate.permits(-0.1, now));
  EXPECT_FALSE(gate.permits(1.1, now));
  EXPECT_FALSE(gate.permits(std::numeric_limits<double>::quiet_NaN(), now));
  gate.observe(0, std::numeric_limits<double>::infinity(), now);
  EXPECT_FALSE(gate.permits(1.0, now));
  EXPECT_TRUE(gate.permits(0.0, now));
  gate.reset_seed();
  EXPECT_FALSE(gate.permits(0.0, now));
}

TEST(ArmSdkMetadataGate, PassiveSeedCannotClaimAfterWaistMoved)
{
  detail::ArmSdkMetadataGate gate;
  const auto now = detail::ArmSdkMetadataGate::Clock::now();
  gate.observe(1, 0.0, now);
  ASSERT_TRUE(gate.seed(now));
  EXPECT_TRUE(gate.prepare_frame(0.0, now));
  gate.observe(1, 0.04, now);
  EXPECT_FALSE(gate.prepare_frame(1.0, now));
  EXPECT_TRUE(gate.prepare_frame(0.0, now));
  ASSERT_TRUE(gate.seed(now));
  EXPECT_TRUE(gate.prepare_frame(0.5, now));
  gate.observe(1, 0.06, now);
  EXPECT_TRUE(gate.prepare_frame(1.0, now));
  EXPECT_FLOAT_EQ(gate.waist_roll(), 0.04F);
}

TEST(UsbSourceIdentity, LoopbackRequiresExplicitUsbAndLanRemainsPrivate)
{
  EXPECT_TRUE(valid_vr_source("127.0.0.1", "usb"));
  EXPECT_FALSE(valid_vr_source("127.0.0.1", "lan"));
  EXPECT_FALSE(valid_vr_source("127.0.0.2", "usb"));
  EXPECT_FALSE(valid_vr_source("192.168.8.42", "usb"));
  EXPECT_TRUE(valid_vr_source("192.168.8.42", "lan"));
  EXPECT_FALSE(valid_vr_source("127.0.0.1", ""));
  EXPECT_FALSE(valid_vr_source("8.8.8.8", "lan"));
}

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

class ArmResponse : public ::testing::Test
{
protected:
  ArmResponseGuard guard;
  ArmResponseGuard::Positions target{};
  ArmResponseGuard::Positions feedback{};
  std::uint64_t sequence{0};

  void SetUp() override {guard.seed(feedback);}
  void send(double time)
  {guard.record_target(target, feedback, sequence, time);}
  void confirm(std::size_t joint, double position, double time)
  {
    feedback[joint] = position;
    for (int sample = 0; sample < 3; ++sample) {
      EXPECT_FALSE(guard.observe(feedback, ++sequence, time + sample * 0.01));
    }
  }
};

TEST_F(ArmResponse, VerifiedRaisedTargetCanHoldWithStaticError)
{
  target[5] = -0.12;
  send(0.0);
  confirm(5, -0.02, 0.1);
  EXPECT_DOUBLE_EQ(guard.command_reference(5), -0.12);
  // A 0.10 rad PD offset is not a new demand for additional motion.
  for (int tick = 0; tick < 400; ++tick) {
    const double time = 0.2 + tick * 0.01;
    send(time);
    EXPECT_FALSE(guard.observe(feedback, ++sequence, time));
    EXPECT_FALSE(guard.pending());
  }
}

TEST_F(ArmResponse, InitialIgnoredCommandTimesOut)
{
  target[5] = -0.12;
  send(0.0);
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 1.49));
  const auto failure = guard.observe(feedback, ++sequence, 1.50);
  ASSERT_TRUE(failure);
  EXPECT_EQ(failure->joint, 5U);
  EXPECT_DOUBLE_EQ(failure->target, -0.12);
  EXPECT_DOUBLE_EQ(failure->command_reference, 0.0);
  EXPECT_EQ(failure->progress_samples, 0);
}

TEST_F(ArmResponse, AlreadyReachedSlowlyAccumulatedGoalDoesNotDemandOvershoot)
{
  for (int step = 1; step <= 81; ++step) {
    target[5] = -0.001 * step;
    feedback = target;
    ++sequence;
    send(step * 0.01);
    EXPECT_FALSE(guard.observe(feedback, sequence, step * 0.01));
  }
  for (int step = 82; step <= 250; ++step) {
    ++sequence;
    send(step * 0.01);
    EXPECT_FALSE(guard.observe(feedback, sequence, step * 0.01));
  }
  EXPECT_FALSE(guard.pending());
}

TEST_F(ArmResponse, ReachedGoalStillRequiresThreeFreshFeedbackSamples)
{
  target[5] = -0.12;
  feedback = target;
  send(0.0);
  EXPECT_FALSE(guard.observe(feedback, 1, 0.1));
  EXPECT_FALSE(guard.observe(feedback, 1, 0.2));
  EXPECT_FALSE(guard.observe(feedback, 2, 0.3));
  EXPECT_TRUE(guard.pending(5));
  EXPECT_FALSE(guard.observe(feedback, 3, 0.4));
  EXPECT_FALSE(guard.pending());
}

TEST_F(ArmResponse, NewIgnoredCommandAfterSuccessfulHoldTimesOut)
{
  target[5] = -0.12;
  send(0.0);
  confirm(5, -0.02, 0.1);
  target[5] = -0.23;
  send(1.0);
  const auto failure = guard.observe(feedback, ++sequence, 2.5);
  ASSERT_TRUE(failure);
  EXPECT_DOUBLE_EQ(failure->command_reference, -0.12);
  EXPECT_DOUBLE_EQ(failure->feedback_baseline, -0.02);
}

TEST_F(ArmResponse, SmallChangesAccumulateAgainstVerifiedCommand)
{
  for (int step = 1; step <= 3; ++step) {
    target[5] = -0.02 * step;
    send(step * 0.1);
    EXPECT_FALSE(guard.pending());
  }
  target[5] = -0.081;
  send(0.5);
  EXPECT_TRUE(guard.pending(5));
  EXPECT_TRUE(guard.observe(feedback, ++sequence, 2.0));
}

TEST_F(ArmResponse, WrongDirectionAndNoiseDoNotConfirmResponse)
{
  target[5] = -0.12;
  send(0.0);
  feedback[5] = 0.02;
  for (int sample = 0; sample < 3; ++sample) {
    EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.1 + sample * 0.01));
  }
  feedback[5] = -0.003;
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.2));
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.21));
  feedback[5] = -0.001;
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.22));
  feedback[5] = -0.003;
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.23));
  EXPECT_TRUE(guard.pending(5));
  feedback[5] = 0.0;
  EXPECT_TRUE(guard.observe(feedback, ++sequence, 1.5));
}

TEST_F(ArmResponse, DuplicateAndOlderFeedbackSequencesDoNotCount)
{
  target[5] = -0.12;
  sequence = 10;
  send(0.0);
  feedback[5] = -0.003;
  EXPECT_FALSE(guard.observe(feedback, 11, 0.1));
  EXPECT_FALSE(guard.observe(feedback, 11, 0.2));
  EXPECT_FALSE(guard.observe(feedback, 10, 0.3));
  EXPECT_FALSE(guard.observe(feedback, 11, 0.4));
  const auto failure = guard.observe(feedback, 11, 1.5);
  ASSERT_TRUE(failure);
  EXPECT_EQ(failure->progress_samples, 1);
}

TEST_F(ArmResponse, DirectionChangeRequiresNewDirectionalEvidence)
{
  target[5] = -0.12;
  send(0.0);
  feedback[5] = -0.003;
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.1));
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.11));
  target[5] = 0.12;
  send(0.2);
  // Progress toward the cancelled negative target cannot prove positive motion.
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.3));
  EXPECT_TRUE(guard.pending(5));
  confirm(5, 0.0, 0.4);
  EXPECT_FALSE(guard.pending(5));
  EXPECT_DOUBLE_EQ(guard.command_reference(5), 0.12);
}

TEST_F(ArmResponse, EachJointKeepsItsOwnResponseObligationAndReference)
{
  target[0] = 0.12;
  target[5] = -0.12;
  send(0.0);
  confirm(0, 0.02, 0.1);
  EXPECT_FALSE(guard.pending(0));
  EXPECT_TRUE(guard.pending(5));
  EXPECT_DOUBLE_EQ(guard.command_reference(0), 0.12);
  EXPECT_DOUBLE_EQ(guard.command_reference(5), 0.0);
  const auto failure = guard.observe(feedback, ++sequence, 1.5);
  ASSERT_TRUE(failure);
  EXPECT_EQ(failure->joint, 5U);
}

TEST_F(ArmResponse, OldProgressDoesNotAcknowledgeLaterLargerTarget)
{
  target[5] = -0.09;
  send(0.0);
  feedback[5] = -0.002;
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.1));
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.11));
  target[5] = -0.21;
  send(0.12);
  EXPECT_FALSE(guard.observe(feedback, ++sequence, 0.13));
  EXPECT_DOUBLE_EQ(guard.command_reference(5), -0.09);
  send(0.14);
  EXPECT_TRUE(guard.pending(5));
  const auto failure = guard.observe(feedback, ++sequence, 1.65);
  ASSERT_TRUE(failure);
  EXPECT_DOUBLE_EQ(failure->observation_target, -0.21);
}

TEST_F(ArmResponse, WithdrawnTargetDoesNotAcknowledgeLargerOldDemand)
{
  target[5] = -0.21;
  send(0.0);
  target[5] = -0.09;
  send(0.1);
  confirm(5, -0.002, 0.2);
  EXPECT_DOUBLE_EQ(guard.command_reference(5), -0.09);
  target[5] = -0.21;
  send(0.3);
  EXPECT_TRUE(guard.pending(5));
}

TEST_F(ArmResponse, ResetRequiresFreshSeedAndForgetsPreviousAuthority)
{
  target[5] = -0.12;
  send(0.0);
  confirm(5, -0.02, 0.1);
  guard.clear();
  EXPECT_FALSE(guard.pending());
  EXPECT_THROW(send(0.2), std::logic_error);
  guard.seed(feedback);
  send(0.3);
  EXPECT_TRUE(guard.pending(5));
  EXPECT_TRUE(guard.observe(feedback, ++sequence, 1.8));
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

TEST(Limits, ArmClampsMagnitudeAndRate)
{
  const auto result = clamp_arm(
    {1.0, -1.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    0.10, 0.25, 1.0);
  EXPECT_DOUBLE_EQ(result[0], 0.10);
  EXPECT_DOUBLE_EQ(result[1], -0.10);
  EXPECT_DOUBLE_EQ(result[2], 0.10);

  const auto large_step = clamp_arm(
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    1.0, 0.25, 10.0);
  EXPECT_DOUBLE_EQ(large_step[0], 0.25);
}

TEST(ArmNeutralHold, AbsoluteCalibratedPoseSurvivesNewWalkingSeed)
{
  ArmNeutralHold hold;
  ArmNeutralHold::Positions seed{};
  ArmNeutralHold::Positions target{};
  seed.fill(0.2);
  target.fill(0.5);
  hold.arm(seed);
  EXPECT_EQ(hold.target_for(target, true), seed);
  ASSERT_TRUE(hold.capture_first_target(target));
  EXPECT_EQ(hold.target_for(target, true), target);
  hold.reset();
  seed.fill(0.3);
  hold.arm(seed);
  EXPECT_EQ(hold.target_for(target, true), seed);
  ASSERT_TRUE(hold.capture_first_target(target));
  EXPECT_EQ(hold.target_for(target, true), target);
  auto invalid = target;
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(hold.target_for(invalid, true), seed);
  const auto limited = clamp_arm(target, seed, 0.02, 0.25, 0.75);
  EXPECT_NEAR(limited[0], 0.315, 1e-9);
}

TEST(ArmNeutralHold, FirstShiftedTargetIsCapturedWithoutDisplacement)
{
  ArmNeutralHold hold;
  const ArmNeutralHold::Positions feedback{0.40, -0.20, 0.10, 0.30, 0.0,
    -0.40, 0.20, -0.10, 0.25, 0.0};
  const ArmNeutralHold::Positions shifted{1.20, -0.90, 0.80, 0.70, 0.4,
    -1.20, 0.90, -0.80, 0.65, -0.4};

  hold.arm(feedback);
  EXPECT_FALSE(hold.neutral_captured());
  EXPECT_TRUE(hold.capture_first_target(shifted));
  EXPECT_TRUE(hold.neutral_captured());
  EXPECT_EQ(hold.target_for(shifted), feedback);
}

TEST(ArmNeutralHold, SubsequentDeltaIsRelativeToCapturedNeutral)
{
  ArmNeutralHold hold;
  const ArmNeutralHold::Positions feedback{0.40, -0.20, 0.10, 0.30, 0.0,
    -0.40, 0.20, -0.10, 0.25, 0.0};
  const ArmNeutralHold::Positions neutral{1.20, -0.90, 0.80, 0.70, 0.4,
    -1.20, 0.90, -0.80, 0.65, -0.4};
  auto moved = neutral;
  moved[0] += 0.15;
  moved[5] -= 0.25;

  hold.arm(feedback);
  ASSERT_TRUE(hold.capture_first_target(neutral));
  const auto target = hold.target_for(moved);
  EXPECT_DOUBLE_EQ(target[0], feedback[0] + 0.15);
  EXPECT_DOUBLE_EQ(target[5], feedback[5] - 0.25);
}

TEST(ArmNeutralHold, ResetAfterReleaseOrKillRequiresFreshNeutral)
{
  ArmNeutralHold hold;
  const ArmNeutralHold::Positions feedback{};
  const ArmNeutralHold::Positions target{1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0};

  hold.arm(feedback);
  ASSERT_TRUE(hold.capture_first_target(target));
  hold.reset();
  EXPECT_FALSE(hold.active());
  EXPECT_FALSE(hold.neutral_captured());
  hold.arm(feedback);
  EXPECT_FALSE(hold.neutral_captured());
  EXPECT_TRUE(hold.capture_first_target(target));
}

TEST(PrepareRearmGate, ExhibitionSessionStillRequiresNeutralVelocity)
{
  PrepareRearmGate gate;
  EXPECT_FALSE(gate.prepared_session(true, {0.02, 0.0, 0.0}));
  EXPECT_FALSE(gate.ready());
  EXPECT_EQ(gate.state(), PrepareRearmState::AwaitNeutralVelocity);

  EXPECT_TRUE(gate.prepared_session(true, {0.0, 0.0, 0.0}));
  EXPECT_TRUE(gate.ready());
}

TEST(PrepareRearmGate, ExhibitionArmsOnlyCanRearmWithoutVelocityStream)
{
  PrepareRearmGate gate;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(gate.prepared_session(false, {nan, nan, nan}));
  EXPECT_TRUE(gate.ready());
}

TEST(Limits, HeadClampsMagnitudeAndRate)
{
  const auto result = clamp_head(
    {9.0, -9.0}, {0.0, 0.0}, 1.0, HeadLimits{});
  EXPECT_DOUBLE_EQ(result[0], 0.035);
  EXPECT_DOUBLE_EQ(result[1], -0.025);
}

TEST(Limits, ExhibitionHeadOneRadianPerSecondRetainsAngleEnvelope)
{
  HeadLimits limits;
  limits.yaw_rate = 1.0;
  limits.pitch_rate = 1.0;
  const auto first = clamp_head({9.0, -9.0}, {0.0, 0.0}, 0.01, limits);
  EXPECT_NEAR(first[0], 0.01, 1e-9);
  EXPECT_NEAR(first[1], -0.01, 1e-9);
  const auto extreme = clamp_head({9.0, -9.0}, {0.0, 0.0}, 1.0, limits);
  EXPECT_LE(extreme[0], limits.yaw);
  EXPECT_GE(extreme[1], -limits.pitch);
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
  // The physical R1 ArmSdk release is one second at 100 Hz.
  constexpr std::size_t steps = 100;
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
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(-0.1, 1, 100)));
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(1.1, 1, 100)));
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(
    std::numeric_limits<double>::quiet_NaN(), 1, 100)));
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(0.5, 1, 0)));
  EXPECT_TRUE(std::isnan(detail::descending_arm_release_weight(0.5, 101, 100)));
}

TEST(StaticStopFallback, RequiresNoLocomotionOrVelocityAmbiguity)
{
  EXPECT_TRUE(detail::static_stop_fallback_eligible(false, false));
  EXPECT_FALSE(detail::static_stop_fallback_eligible(true, false));
  EXPECT_FALSE(detail::static_stop_fallback_eligible(false, true));
  EXPECT_FALSE(detail::static_stop_fallback_eligible(true, true));
}

TEST(StaticStopFallback, AcceptsOnlySuccessfulFsmFourSamples)
{
  EXPECT_TRUE(detail::confirms_static_standing_sample(0, 4));
  EXPECT_FALSE(detail::confirms_static_standing_sample(127, 4));
  EXPECT_FALSE(detail::confirms_static_standing_sample(0, 811));
  EXPECT_FALSE(detail::confirms_static_standing_sample(0, -1));
}

TEST(NoVelocityStopFallback, RejectsAnyVelocityBoundary)
{
  EXPECT_TRUE(detail::no_velocity_stop_fallback_eligible(false));
  EXPECT_FALSE(detail::no_velocity_stop_fallback_eligible(true));
}

TEST(NoVelocityStopFallback, ConfirmsOnlyTheEnteredNonmovingMode)
{
  EXPECT_TRUE(detail::confirms_nonmoving_mode_sample(0, 4, false));
  EXPECT_FALSE(detail::confirms_nonmoving_mode_sample(0, 811, false));
  EXPECT_TRUE(detail::confirms_nonmoving_mode_sample(0, 811, true));
  EXPECT_FALSE(detail::confirms_nonmoving_mode_sample(0, 4, true));
  EXPECT_FALSE(detail::confirms_nonmoving_mode_sample(127, 811, true));
}

TEST(WirelessControllerMapping, ConvertsReviewedVelocityCeilingsToAxes)
{
  const auto command = detail::velocity_to_wireless_controller(0.15, 0.12, 0.35);
  EXPECT_FLOAT_EQ(command.lx, -1.0F);
  EXPECT_FLOAT_EQ(command.ly, 0.75F);
  EXPECT_FLOAT_EQ(command.rx, -1.0F);
  EXPECT_FLOAT_EQ(command.ry, 0.0F);
  EXPECT_EQ(command.keys, 0U);
}

TEST(WirelessControllerMapping, ClampsAxesAndKeepsNeutralFrameZero)
{
  const auto clamped = detail::velocity_to_wireless_controller(-2.0, 2.0, -2.0);
  EXPECT_FLOAT_EQ(clamped.lx, -1.0F);
  EXPECT_FLOAT_EQ(clamped.ly, -1.0F);
  EXPECT_FLOAT_EQ(clamped.rx, 1.0F);
  EXPECT_FLOAT_EQ(clamped.ry, 0.0F);
  EXPECT_EQ(clamped.keys, 0U);

  const auto zero = detail::velocity_to_wireless_controller(0.0, 0.0, 0.0);
  EXPECT_FLOAT_EQ(zero.lx, 0.0F);
  EXPECT_FLOAT_EQ(zero.ly, 0.0F);
  EXPECT_FLOAT_EQ(zero.rx, 0.0F);
  EXPECT_FLOAT_EQ(zero.ry, 0.0F);
  EXPECT_EQ(zero.keys, 0U);
}

TEST(WirelessControllerMapping, LeftAndRightRosLateralMatchPhysicalPilotDirection)
{
  // Positive ROS Y is left, while positive R1 pilot LX moved this robot right.
  const auto left = detail::velocity_to_wireless_controller(0.0, 0.06, 0.0);
  const auto right = detail::velocity_to_wireless_controller(0.0, -0.06, 0.0);
  EXPECT_FLOAT_EQ(left.lx, -0.5F);
  EXPECT_FLOAT_EQ(right.lx, 0.5F);
  EXPECT_FLOAT_EQ(left.ly, 0.0F);
  EXPECT_FLOAT_EQ(left.rx, 0.0F);
  EXPECT_EQ(left.keys, 0U);
  EXPECT_EQ(right.keys, 0U);
}

TEST(WirelessControllerMapping, RightAndLeftRosYawMatchPhysicalPilotDirection)
{
  // Right VR X -> negative ROS yaw -> positive pilot RX (right turn).
  const auto right = detail::velocity_to_wireless_controller(0.0, 0.0, -0.175);
  const auto left = detail::velocity_to_wireless_controller(0.0, 0.0, 0.175);
  EXPECT_FLOAT_EQ(right.rx, 0.5F);
  EXPECT_FLOAT_EQ(left.rx, -0.5F);
  EXPECT_FLOAT_EQ(right.lx, 0.0F);
  EXPECT_FLOAT_EQ(right.ly, 0.0F);
  EXPECT_EQ(right.keys, 0U);
  EXPECT_EQ(left.keys, 0U);
}

TEST(WirelessControllerMapping, NonfiniteInputFailsToNeutral)
{
  const auto command = detail::velocity_to_wireless_controller(
    std::numeric_limits<double>::quiet_NaN(), 0.1, 0.1);
  EXPECT_FLOAT_EQ(command.lx, 0.0F);
  EXPECT_FLOAT_EQ(command.ly, 0.0F);
  EXPECT_FLOAT_EQ(command.rx, 0.0F);
  EXPECT_FLOAT_EQ(command.ry, 0.0F);
  EXPECT_EQ(command.keys, 0U);
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

TEST(MockTransport, ArmsRunningEntersControllerWithoutVelocityPipeline)
{
  MockTransport transport;
  ASSERT_TRUE(transport.initialize("mock0", true, false).ok);

  const TransportResult prepare = transport.prepare([]() {return false;}, true);
  ASSERT_TRUE(prepare.ok);
  EXPECT_EQ(transport.prepare_calls(), 1U);

  // FSM 811 entry is allowed for the arm-only commissioning stage, but the
  // disabled locomotion feature must still reject every joystick sample.
  EXPECT_FALSE(transport.set_velocity(0.1, 0.0, 0.0, 0.12).ok);

  // STOP/KILL cleanup remains callable after controller entry even when no
  // SetVelocity frame was ever sent.
  EXPECT_TRUE(transport.stop_locomotion().ok);
  EXPECT_EQ(transport.velocity_calls(), 0U);
  EXPECT_EQ(transport.stop_calls(), 1U);
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
