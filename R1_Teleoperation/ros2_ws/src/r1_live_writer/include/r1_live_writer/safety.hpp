#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace r1_live_writer
{

struct LiveEnvironment
{
  bool dry_run_disabled{false};
  bool actuation_enabled{false};
  bool off_charger_confirmed{false};
  bool clear_area_confirmed{false};
  bool estop_ready_confirmed{false};
  bool commissioning_confirmed{false};
  std::string commissioning_token;
  std::string vr_source_ip;
  std::string vr_transport{"lan"};

  static LiveEnvironment from_process();
  std::vector<std::string> missing(bool require_vr_source = true) const;
};

struct AuthorizationInput
{
  bool send_commands{false};
  bool feature_enabled{false};
  bool commissioning_parameter_confirmed{false};
  bool commissioning_token_matches{false};
  bool require_vr_source{true};
  bool vr_source_matches{false};
  bool profile_is_slow_safe{false};
  bool kill_clear_fresh{false};
  bool deadman_active_fresh{false};
  bool state_fresh{false};
  // Required only for a physical SDK/send_commands session. Mock and dry-arm
  // use remain independent of physical LowState motor fault telemetry.
  bool motor_health_required{false};
  bool motors_healthy_fresh{false};
  bool command_fresh{false};
  bool prepared{false};
  bool require_prepared{true};
  LiveEnvironment environment;
};

struct GateDecision
{
  bool allowed{false};
  std::string reason;
};

GateDecision authorize_live_send(const AuthorizationInput & input);
bool valid_vr_source(const std::string & text, const std::string & transport);

struct VelocityLimits
{
  double forward{0.20};
  double lateral{0.12};
  double yaw{0.35};
  double linear_rate{0.25};
  double yaw_rate{0.50};
};

std::array<double, 3> clamp_velocity(
  const std::array<double, 3> & requested,
  const std::array<double, 3> & previous,
  double dt,
  const VelocityLimits & limits);

std::array<double, 10> clamp_arm(
  const std::array<double, 10> & requested,
  const std::array<double, 10> & previous,
  double dt,
  double max_delta,
  double max_rate);

// Re-arm helper for absolute VR arm targets.  The first target observed after
// a stop/kill is captured as the controller neutral and never sent as a
// physical displacement.  Later targets are expressed as deltas from that
// neutral and added to the fresh physical feedback seed.
class ArmNeutralHold
{
public:
  using Positions = std::array<double, 10>;

  void reset();
  void arm(const Positions & physical_feedback);
  bool active() const {return active_;}
  bool neutral_captured() const {return neutral_captured_;}
  bool capture_first_target(const Positions & vr_target);
  Positions target_for(const Positions & vr_target, bool absolute = false) const;
  const Positions & physical_feedback() const {return physical_feedback_;}

private:
  bool active_{false};
  bool neutral_captured_{false};
  Positions physical_feedback_{};
  Positions vr_neutral_{};
};

// Verify a response to each joint's new command, not convergence to its absolute
// position. PD/gravity error may remain after the robot has visibly responded.
class ArmResponseGuard
{
public:
  using Positions = std::array<double, 10>;
  static constexpr int kRequiredProgressSamples = 3;

  struct Failure
  {
    std::size_t joint;
    double direction;
    double target;
    double observation_target;
    double command_reference;
    double feedback_baseline;
    double feedback;
    int progress_samples;
    double elapsed_sec;
  };

  explicit ArmResponseGuard(
    double command_delta_rad = 0.08, double min_progress_rad = 0.0015,
    double timeout_sec = 1.50);
  void clear();
  void seed(const Positions & commanded_feedback);
  void record_target(
    const Positions & target, const Positions & feedback,
    std::uint64_t feedback_sequence, double now_sec);
  std::optional<Failure> observe(
    const Positions & feedback, std::uint64_t feedback_sequence, double now_sec);
  bool pending() const;
  bool pending(std::size_t joint) const {return joints_.at(joint).pending;}
  double command_reference(std::size_t joint) const
  {return joints_.at(joint).command_reference;}

private:
  struct Joint
  {
    double command_reference{0.0};
    double target{0.0};
    double observation_target{0.0};
    double feedback_baseline{0.0};
    double direction{0.0};
    double started_sec{0.0};
    std::uint64_t last_sequence{0};
    int progress_samples{0};
    bool pending{false};
  };

  double command_delta_rad_;
  double min_progress_rad_;
  double timeout_sec_;
  bool seeded_{false};
  std::array<Joint, 10> joints_{};
};

struct HeadLimits
{
  // Headset-relative excursion and slew limits.  These are deliberately
  // separate from the robot's absolute joint envelope below.
  double yaw{0.35};
  double pitch{0.25};
  double yaw_rate{0.35};
  double pitch_rate{0.25};
};

std::array<double, 2> clamp_head(
  const std::array<double, 2> & requested_yaw_pitch,
  const std::array<double, 2> & previous_yaw_pitch,
  double dt,
  const HeadLimits & limits);

struct HeadAbsoluteLimits
{
  double yaw_min{-2.0071};
  double yaw_max{2.0071};
  double pitch_min{-0.6283};
  double pitch_max{0.6283};
};

bool head_within_absolute_limits(
  const std::array<double, 2> & yaw_pitch,
  const HeadAbsoluteLimits & limits);

std::array<double, 2> absolute_head_target(
  const std::array<double, 2> & origin_yaw_pitch,
  const std::array<double, 2> & relative_delta_yaw_pitch,
  const HeadAbsoluteLimits & limits);

std::array<double, 2> step_head_toward_zero(
  const std::array<double, 2> & previous_yaw_pitch,
  double dt,
  double yaw_rate,
  double pitch_rate);

bool finite_values(const std::array<double, 3> & values);
bool finite_values(const std::array<double, 2> & values);
bool sane_arm_sdk_feedback(const std::array<double, 13> & positions);
bool valid_private_unicast_ipv4(const std::string & text);

enum class PrepareRearmState
{
  Unprepared,
  AwaitDeadmanRelease,
  AwaitNeutralVelocity,
  AwaitDeadmanPress,
  Ready,
};

class PrepareRearmGate
{
public:
  explicit PrepareRearmGate(double neutral_velocity_epsilon = 0.01);

  void prepared(bool locomotion_enabled);
  // Exhibition control is armed by one explicit panel action rather than a
  // continuously held button.  Reuse the same post-prepare state machine and
  // retain its neutral-stick requirement, but synthesize the release/press
  // edges inside the writer after Stand/FSM confirmation.  This entry point
  // is never used by the normal Deadman path.
  bool prepared_session(
    bool locomotion_enabled,
    const std::array<double, 3> & neutral_velocity);
  void reset();
  bool observe_deadman(bool active);
  bool observe_velocity(const std::array<double, 3> & velocity);
  bool ready() const {return state_ == PrepareRearmState::Ready;}
  PrepareRearmState state() const {return state_;}
  std::string state_name() const;

private:
  double neutral_velocity_epsilon_{0.01};
  bool locomotion_enabled_{false};
  bool deadman_active_{false};
  PrepareRearmState state_{PrepareRearmState::Unprepared};
};

}  // namespace r1_live_writer
