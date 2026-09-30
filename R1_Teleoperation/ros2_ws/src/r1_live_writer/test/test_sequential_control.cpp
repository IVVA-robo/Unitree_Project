#include <gtest/gtest.h>
#include "r1_live_writer/sequential_control.hpp"
#include "r1_live_writer/transport.hpp"

using r1_live_writer::SequentialControl;
using Phase = SequentialControl::Phase;

struct SequenceTest : testing::Test
{
  SequentialControl policy;
  SequentialControl::Input input;
  r1_live_writer::MockTransport transport;
  r1_live_writer::ArmSdkPositions pose{};
  void SetUp() override
  {
    ASSERT_TRUE(transport.initialize("mock", true, true).ok);
    ASSERT_TRUE(transport.seed_head(pose).ok);
    input.tracking = true;
    input.owns_upper = true;
    input.mode_valid = true;
    input.mode = 816;
    input.now = 10;
  }
  void tick(double delta = 0.01)
  {
    input.now += delta;
    ++input.state_sequence;
    policy.update(input);
  }
  void request_walk()
  {
    input.motion = true;
    tick(); tick(0.09);
    ASSERT_EQ(policy.phase(), Phase::Release);
  }
  void release()
  {
    for (int frame = 0; frame < 130; ++frame) {
      input.now += 0.0101;
      ASSERT_TRUE(policy.release_due(input.now));
      ASSERT_TRUE(transport.command_head_weighted(pose, policy.release_weight()).ok);
      policy.release_frame_sent(input.now);
      EXPECT_NE(policy.phase(), Phase::Walk);
    }
    EXPECT_FALSE(policy.release_complete(input.now));
    input.now += 0.011;
    ASSERT_TRUE(policy.release_complete(input.now));
    ASSERT_TRUE(transport.finish_head_handover().ok);
    input.owns_upper = false;
    policy.release_finished(input.now);
  }
  void walking()
  {
    request_walk(); release();
    input.mode = 811;
    input.mode_requested_at = input.now + 0.001;
    tick();
    ASSERT_EQ(policy.phase(), Phase::Walk);
  }
};

TEST_F(SequenceTest, StickNoiseDoesNotReleaseArms)
{
  for (int n = 0; n < 20; ++n) {
    input.motion = true; tick(0.02);
    input.motion = false; tick(0.02);
  }
  EXPECT_EQ(policy.phase(), Phase::Upper);
}

TEST_F(SequenceTest, VendorRampIsMonotoneAndDoesNotPublishVelocity)
{
  request_walk();
  double previous = 1;
  const double started = input.now;
  for (int n = 0; n < 130; ++n) {
    input.now += 0.0101;
    const double weight = policy.release_weight();
    EXPECT_LE(weight, previous);
    EXPECT_GE(weight, 0);
    previous = weight;
    ASSERT_TRUE(transport.command_head_weighted(pose, weight).ok);
    policy.release_frame_sent(input.now);
  }
  EXPECT_EQ(transport.last_head_weight(), 0);
  EXPECT_EQ(transport.velocity_calls(), 0U);
  EXPECT_LT(input.now - started, 1.4);
  EXPECT_FALSE(policy.release_due(input.now + 1));
}

TEST_F(SequenceTest, Old811SampleCannotAuthorizeWalking)
{
  request_walk();
  input.mode = 811;
  input.mode_requested_at = input.now;
  release(); tick();
  EXPECT_EQ(policy.phase(), Phase::AwaitWalk);
  input.mode_requested_at = input.now;
  tick();
  EXPECT_EQ(policy.phase(), Phase::Walk);
}

TEST_F(SequenceTest, ReleaseRequiresPhysicalModeConfirmation)
{
  request_walk(); release(); tick(0.3);
  EXPECT_EQ(policy.phase(), Phase::AwaitWalk);
  tick(3.0);
  EXPECT_EQ(policy.phase(), Phase::Fault);
}

TEST_F(SequenceTest, ReleaseTimeoutDoesNotAllowMotion)
{
  request_walk(); tick(3.1);
  EXPECT_EQ(policy.phase(), Phase::Fault);
}

TEST_F(SequenceTest, LettingGoDuringReleaseDoesNotReplayOldStick)
{
  request_walk(); input.motion = false; tick(); release();
  input.mode = 811;
  input.mode_requested_at = input.now;
  tick();
  EXPECT_EQ(policy.phase(), Phase::Stop);
}

TEST_F(SequenceTest, StopRequiresFeedbackAndRepeatedZeros)
{
  walking(); input.motion = false; tick();
  EXPECT_EQ(policy.phase(), Phase::Stop);
  input.mode_requested_at = input.now;
  for (int i = 0; i < 8; ++i) {policy.zero_sent(); tick(0.05);}
  EXPECT_EQ(policy.phase(), Phase::Stop);  // legs still moving
  input.legs_quiet = true;
  for (int i = 0; i < 7; ++i) {tick(0.05);}
  EXPECT_EQ(policy.phase(), Phase::Upper);
  EXPECT_FALSE(input.owns_upper);  // node must freshly seed, never resume old q
}

TEST_F(SequenceTest, QuietFeedbackWithoutZerosDoesNotReclaim)
{
  walking(); input.motion = false; tick();
  input.mode_requested_at = input.now;
  input.legs_quiet = true;
  for (int i = 0; i < 10; ++i) {tick(0.05);}
  EXPECT_EQ(policy.phase(), Phase::Stop);
}

TEST_F(SequenceTest, RepeatedSameFeedbackCannotProveStopped)
{
  walking(); input.motion = false; tick();
  input.mode_requested_at = input.now;
  input.legs_quiet = true;
  for (int i = 0; i < 10; ++i) {
    policy.zero_sent(); input.now += 0.05; policy.update(input);
  }
  EXPECT_EQ(policy.phase(), Phase::Stop);
}

TEST_F(SequenceTest, TrackingLossZerosBeforeReconnect)
{
  walking(); input.tracking = false; tick();
  EXPECT_EQ(policy.phase(), Phase::Stop);
  input.mode_requested_at = input.now;
  tick(0.2);
  EXPECT_EQ(policy.phase(), Phase::Stop);
  input.motion = false; input.tracking = true; tick();
  EXPECT_EQ(policy.phase(), Phase::Stop);
}

TEST_F(SequenceTest, ModeLossStopsWalking)
{
  walking(); input.mode_valid = false; tick();
  EXPECT_EQ(policy.phase(), Phase::Stop);
  tick(5.1);
  EXPECT_EQ(policy.phase(), Phase::Fault);
}

TEST_F(SequenceTest, RenewedStickCanReuseWalkingWithoutArmClaim)
{
  walking(); input.motion = false; tick();
  input.mode_requested_at = input.now;
  input.motion = true; tick(); tick(0.09);
  EXPECT_EQ(policy.phase(), Phase::Walk);
  EXPECT_EQ(transport.head_seed_calls(), 1U);
}

TEST_F(SequenceTest, ResetDropsEveryPendingTransition)
{
  request_walk(); release();
  policy.reset();
  input.motion = false; input.tracking = false; tick();
  EXPECT_EQ(policy.phase(), Phase::Upper);
  EXPECT_FALSE(policy.release_due(input.now));
  EXPECT_FALSE(policy.release_complete(input.now));
}

TEST_F(SequenceTest, TransportCannotFinishPositiveOwnership)
{
  EXPECT_FALSE(transport.finish_head_handover().ok);
  ASSERT_TRUE(transport.command_head_weighted(pose, 0).ok);
  ASSERT_TRUE(transport.finish_head_handover().ok);
  EXPECT_FALSE(transport.command_head(pose).ok);
  EXPECT_TRUE(transport.seed_head(pose).ok);
}
