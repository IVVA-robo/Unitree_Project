#pragma once

#include <array>
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

  static LiveEnvironment from_process();
  std::vector<std::string> missing() const;
};

struct AuthorizationInput
{
  bool send_commands{false};
  bool feature_enabled{false};
  bool commissioning_parameter_confirmed{false};
  bool commissioning_token_matches{false};
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
