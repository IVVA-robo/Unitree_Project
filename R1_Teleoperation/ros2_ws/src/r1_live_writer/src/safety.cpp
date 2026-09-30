#include "r1_live_writer/safety.hpp"

#include <arpa/inet.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace r1_live_writer
{
namespace
{

bool env_is(const char * name, const char * expected)
{
  const char * value = std::getenv(name);
  return value != nullptr && std::string(value) == expected;
}

std::string env_text(const char * name)
{
  const char * value = std::getenv(name);
  return value == nullptr ? std::string{} : std::string(value);
}

double bounded_axis(double requested, double previous, double magnitude, double rate, double dt)
{
  const double target = std::clamp(requested, -magnitude, magnitude);
  const double max_step = rate * std::clamp(dt, 0.0, 0.10);
  return previous + std::clamp(target - previous, -max_step, max_step);
}

}  // namespace

LiveEnvironment LiveEnvironment::from_process()
{
  LiveEnvironment result;
  result.dry_run_disabled = env_is("ROBOT_DRY_RUN", "0");
  result.actuation_enabled = env_is("ROBOT_ENABLE_ACTUATION", "1");
  result.off_charger_confirmed = env_is("ROBOT_CONFIRM_OFF_CHARGER", "1");
  result.clear_area_confirmed = env_is("ROBOT_CONFIRM_CLEAR_AREA", "1");
  result.estop_ready_confirmed = env_is("ROBOT_CONFIRM_ESTOP_READY", "1");
  result.commissioning_confirmed = env_is("ROBOT_CONFIRM_COMMISSIONING", "1");
  result.commissioning_token = env_text("ROBOT_COMMISSIONING_TOKEN");
  result.vr_source_ip = env_text("ROBOT_VR_SOURCE_IP");
  const auto transport = env_text("R1_VR_TRANSPORT");
  result.vr_transport = transport.empty() ? "lan" : transport;
  return result;
}

std::vector<std::string> LiveEnvironment::missing(bool require_vr_source) const
{
  std::vector<std::string> result;
  if (!dry_run_disabled) {
    result.emplace_back("ROBOT_DRY_RUN=0");
  }
  if (!actuation_enabled) {
    result.emplace_back("ROBOT_ENABLE_ACTUATION=1");
  }
  if (!off_charger_confirmed) {
    result.emplace_back("ROBOT_CONFIRM_OFF_CHARGER=1");
  }
  if (!clear_area_confirmed) {
    result.emplace_back("ROBOT_CONFIRM_CLEAR_AREA=1");
  }
  if (!estop_ready_confirmed) {
    result.emplace_back("ROBOT_CONFIRM_ESTOP_READY=1");
  }
  if (!commissioning_confirmed) {
    result.emplace_back("ROBOT_CONFIRM_COMMISSIONING=1");
  }
  if (commissioning_token.empty()) {
    result.emplace_back("ROBOT_COMMISSIONING_TOKEN");
  }
  if (require_vr_source && vr_source_ip.empty()) {
    result.emplace_back("ROBOT_VR_SOURCE_IP");
  }
  return result;
}

GateDecision authorize_live_send(const AuthorizationInput & input)
{
  if (!input.send_commands) {
    return {false, "send_commands=false"};
  }
  if (!input.feature_enabled) {
    return {false, "feature_disabled"};
  }
  const auto missing = input.environment.missing(input.require_vr_source);
  if (!missing.empty()) {
    std::ostringstream stream;
    stream << "environment_missing=";
    for (std::size_t index = 0; index < missing.size(); ++index) {
      if (index != 0) {
        stream << ',';
      }
      stream << missing[index];
    }
    return {false, stream.str()};
  }
  if (!input.commissioning_parameter_confirmed) {
    return {false, "commissioning_parameter_not_confirmed"};
  }
  if (!input.commissioning_token_matches) {
    return {false, "commissioning_token_mismatch"};
  }
  if (!input.vr_source_matches) {
    return {false, "vr_source_ip_mismatch"};
  }
  if (!input.profile_is_slow_safe) {
    return {false, "first_live_profile_must_be_slow-safe"};
  }
  if (!input.kill_clear_fresh) {
    return {false, "kill_not_explicitly_clear_and_fresh"};
  }
  if (!input.deadman_active_fresh) {
    return {false, "deadman_not_active_and_fresh"};
  }
  if (!input.state_fresh) {
    return {false, "joint_state_missing_stale_or_invalid"};
  }
  if (input.motor_health_required && !input.motors_healthy_fresh) {
    return {false, "motor_health_missing_stale_or_unhealthy"};
  }
  if (!input.command_fresh) {
    return {false, "command_missing_stale_or_invalid"};
  }
  if (input.require_prepared && !input.prepared) {
    return {false, "robot_not_prepared"};
  }
  return {true, "authorized"};
}

bool finite_values(const std::array<double, 3> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

bool finite_values(const std::array<double, 2> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

bool sane_arm_sdk_feedback(const std::array<double, 13> & positions)
{
  // These fields are copied back as a hold target before either head field is
  // changed. A finite but corrupt encoder value must not be echoed to ArmSdk.
  // This broad envelope contains every arm/waist range in the local R1
  // description; the node applies much smaller first-live limits to the head.
  return std::all_of(positions.begin(), positions.end(), [](double value) {
    return std::isfinite(value) && std::abs(value) <= 3.5;
  });
}

bool valid_private_unicast_ipv4(const std::string & text)
{
  in_addr address{};
  if (text.empty() || inet_pton(AF_INET, text.c_str(), &address) != 1) {
    return false;
  }
  const std::uint32_t host = ntohl(address.s_addr);
  const std::uint8_t first = static_cast<std::uint8_t>(host >> 24U);
  const std::uint8_t second = static_cast<std::uint8_t>((host >> 16U) & 0xffU);
  // Accept a fixed LAN source, including shared-address exhibition networks.
  // This rejects public, loopback, link-local, multicast, unspecified, and
  // broadcast destinations even when they are syntactically valid IPv4.
  return first == 10U ||
         (first == 172U && second >= 16U && second <= 31U) ||
         (first == 192U && second == 168U) ||
         (first == 100U && second >= 64U && second <= 127U);
}

bool valid_vr_source(const std::string & text, const std::string & transport)
{
  // Loopback is an explicit USB relay identity, never a LAN wildcard.
  if (transport == "usb") {
    return text == "127.0.0.1";
  }
  return transport == "lan" && valid_private_unicast_ipv4(text);
}

PrepareRearmGate::PrepareRearmGate(double neutral_velocity_epsilon)
: neutral_velocity_epsilon_(neutral_velocity_epsilon)
{
  if (!std::isfinite(neutral_velocity_epsilon_) ||
    neutral_velocity_epsilon_ < 0.0 || neutral_velocity_epsilon_ > 0.02)
  {
    throw std::invalid_argument(
            "neutral velocity epsilon must be finite and in [0, 0.02]");
  }
}

void PrepareRearmGate::prepared(bool locomotion_enabled)
{
  locomotion_enabled_ = locomotion_enabled;
  // prepare() itself requires a held Deadman. A release observed after this
  // method is therefore an explicit post-prepare edge, not a stale state.
  deadman_active_ = true;
  state_ = PrepareRearmState::AwaitDeadmanRelease;
}

bool PrepareRearmGate::prepared_session(
  bool locomotion_enabled,
  const std::array<double, 3> & neutral_velocity)
{
  prepared(locomotion_enabled);
  (void)observe_deadman(false);
  if (locomotion_enabled && !observe_velocity(neutral_velocity)) {
    return false;
  }
  return observe_deadman(true) && ready();
}

void PrepareRearmGate::reset()
{
  locomotion_enabled_ = false;
  deadman_active_ = false;
  state_ = PrepareRearmState::Unprepared;
}

bool PrepareRearmGate::observe_deadman(bool active)
{
  deadman_active_ = active;
  if (!active) {
    if (state_ == PrepareRearmState::AwaitDeadmanRelease ||
      state_ == PrepareRearmState::Ready)
    {
      state_ = locomotion_enabled_ ?
        PrepareRearmState::AwaitNeutralVelocity :
        PrepareRearmState::AwaitDeadmanPress;
    }
    return false;
  }
  if (state_ == PrepareRearmState::AwaitDeadmanPress) {
    state_ = PrepareRearmState::Ready;
    return true;
  }
  return false;
}

bool PrepareRearmGate::observe_velocity(const std::array<double, 3> & velocity)
{
  if (state_ != PrepareRearmState::AwaitNeutralVelocity || deadman_active_ ||
    !finite_values(velocity))
  {
    return false;
  }
  const bool neutral = std::all_of(velocity.begin(), velocity.end(), [this](double value) {
    return std::abs(value) <= neutral_velocity_epsilon_;
  });
  if (!neutral) {
    return false;
  }
  state_ = PrepareRearmState::AwaitDeadmanPress;
  return true;
}

std::string PrepareRearmGate::state_name() const
{
  switch (state_) {
    case PrepareRearmState::Unprepared:
      return "unprepared";
    case PrepareRearmState::AwaitDeadmanRelease:
      return "await_deadman_release";
    case PrepareRearmState::AwaitNeutralVelocity:
      return "await_neutral_velocity";
    case PrepareRearmState::AwaitDeadmanPress:
      return "await_deadman_press";
    case PrepareRearmState::Ready:
      return "ready";
  }
  return "invalid";
}

std::array<double, 3> clamp_velocity(
  const std::array<double, 3> & requested,
  const std::array<double, 3> & previous,
  double dt,
  const VelocityLimits & limits)
{
  if (!finite_values(requested) || !finite_values(previous) || !std::isfinite(dt)) {
    return {0.0, 0.0, 0.0};
  }
  return {
    bounded_axis(requested[0], previous[0], limits.forward, limits.linear_rate, dt),
    bounded_axis(requested[1], previous[1], limits.lateral, limits.linear_rate, dt),
    bounded_axis(requested[2], previous[2], limits.yaw, limits.yaw_rate, dt)};
}

std::array<double, 10> clamp_arm(
  const std::array<double, 10> & requested,
  const std::array<double, 10> & previous,
  double dt,
  double max_delta,
  double max_rate)
{
  if (!std::all_of(requested.begin(), requested.end(), [](double value) {
      return std::isfinite(value);
    }) || !std::all_of(previous.begin(), previous.end(), [](double value) {
      return std::isfinite(value);
    }) || !std::isfinite(dt) || !std::isfinite(max_delta) || max_delta <= 0.0 ||
    !std::isfinite(max_rate) || max_rate <= 0.0)
  {
    return previous;
  }
  const double bounded_dt = std::clamp(dt, 0.0, 0.10);
  const double step = std::min(max_delta, max_rate * bounded_dt);
  std::array<double, 10> result = previous;
  for (std::size_t index = 0; index < result.size(); ++index) {
    const double delta = requested[index] - previous[index];
    result[index] = previous[index] + std::clamp(delta, -step, step);
  }
  return result;
}

void ArmNeutralHold::reset()
{
  active_ = false;
  neutral_captured_ = false;
  physical_feedback_.fill(0.0);
  vr_neutral_.fill(0.0);
}

void ArmNeutralHold::arm(const Positions & physical_feedback)
{
  if (!std::all_of(physical_feedback.begin(), physical_feedback.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    reset();
    return;
  }
  active_ = true;
  neutral_captured_ = false;
  physical_feedback_ = physical_feedback;
  vr_neutral_.fill(0.0);
}

bool ArmNeutralHold::capture_first_target(const Positions & vr_target)
{
  if (!active_ || neutral_captured_ ||
    !std::all_of(vr_target.begin(), vr_target.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    return false;
  }
  vr_neutral_ = vr_target;
  neutral_captured_ = true;
  return true;
}

ArmNeutralHold::Positions ArmNeutralHold::target_for(
  const Positions & vr_target, bool absolute) const
{
  if (!active_ || !neutral_captured_ ||
    !std::all_of(vr_target.begin(), vr_target.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    return physical_feedback_;
  }
  // Calibrated joint poses keep the same destination after walking/reacquiring
  // ArmSdk. The caller still rate-limits from the newly measured physical seed.
  if (absolute) {return vr_target;}
  Positions result = physical_feedback_;
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] += vr_target[index] - vr_neutral_[index];
  }
  return result;
}

ArmResponseGuard::ArmResponseGuard(
  double command_delta_rad, double min_progress_rad, double timeout_sec)
: command_delta_rad_(command_delta_rad), min_progress_rad_(min_progress_rad),
  timeout_sec_(timeout_sec)
{
  if (!std::isfinite(command_delta_rad_) || command_delta_rad_ < 0.05 ||
    command_delta_rad_ > 0.15 || !std::isfinite(min_progress_rad_) ||
    min_progress_rad_ < 0.001 || min_progress_rad_ > command_delta_rad_ ||
    !std::isfinite(timeout_sec_) || timeout_sec_ <= 0.0 || timeout_sec_ > 1.50)
  {
    throw std::invalid_argument("invalid arm response guard limits");
  }
}

void ArmResponseGuard::clear()
{
  seeded_ = false;
  joints_ = {};
}

void ArmResponseGuard::seed(const Positions & commanded_feedback)
{
  clear();
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    if (!std::isfinite(commanded_feedback[index])) {
      throw std::invalid_argument("arm response seed must be finite");
    }
    joints_[index].command_reference = commanded_feedback[index];
    joints_[index].target = commanded_feedback[index];
  }
  seeded_ = true;
}

void ArmResponseGuard::record_target(
  const Positions & target, const Positions & feedback,
  std::uint64_t feedback_sequence, double now_sec)
{
  if (!seeded_) {
    throw std::logic_error("arm response guard requires a physical command seed");
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    auto & joint = joints_[index];
    joint.target = target[index];
    const double delta = target[index] - joint.command_reference;
    if (std::abs(delta) < command_delta_rad_) {
      // A withdrawn command no longer requires a response. Keep the verified
      // reference so small later changes accumulate rather than evading checks.
      joint.pending = false;
      joint.progress_samples = 0;
      continue;
    }
    const double direction = delta > 0.0 ? 1.0 : -1.0;
    if (!joint.pending || direction != joint.direction) {
      joint.pending = true;
      joint.direction = direction;
      joint.started_sec = now_sec;
      joint.feedback_baseline = feedback[index];
      joint.last_sequence = feedback_sequence;
      joint.progress_samples = 0;
      joint.observation_target = target[index];
    } else if (direction * (target[index] - joint.observation_target) < 0.0) {
      // A smaller demand can replace the observed demand, but a later larger
      // target must never inherit proof from movement elicited by an older one.
      joint.observation_target = target[index];
    }
  }
}

std::optional<ArmResponseGuard::Failure> ArmResponseGuard::observe(
  const Positions & feedback, std::uint64_t feedback_sequence, double now_sec)
{
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    auto & joint = joints_[index];
    if (!joint.pending) {
      continue;
    }
    if (feedback_sequence > joint.last_sequence) {
      joint.last_sequence = feedback_sequence;
      const double progress =
        (feedback[index] - joint.feedback_baseline) * joint.direction;
      // With slowly accumulated commands the joint can already have reached
      // this goal before the observation window starts. Three fresh samples
      // at that goal prove response without demanding motion past the goal.
      const bool at_observed_goal =
        std::abs(feedback[index] - joint.observation_target) <= min_progress_rad_;
      joint.progress_samples = progress >= min_progress_rad_ || at_observed_goal ?
        joint.progress_samples + 1 : 0;
    }
    if (joint.progress_samples >= kRequiredProgressSamples) {
      joint.command_reference = joint.observation_target;
      joint.pending = false;
      joint.progress_samples = 0;
      continue;
    }
    if (now_sec - joint.started_sec >= timeout_sec_) {
      return Failure{
        index, joint.direction, joint.target, joint.observation_target,
        joint.command_reference, joint.feedback_baseline, feedback[index],
        joint.progress_samples, now_sec - joint.started_sec};
    }
  }
  return std::nullopt;
}

bool ArmResponseGuard::pending() const
{
  return std::any_of(joints_.begin(), joints_.end(), [](const Joint & joint) {
      return joint.pending;
    });
}

std::array<double, 2> clamp_head(
  const std::array<double, 2> & requested_yaw_pitch,
  const std::array<double, 2> & previous_yaw_pitch,
  double dt,
  const HeadLimits & limits)
{
  if (!finite_values(requested_yaw_pitch) ||
    !finite_values(previous_yaw_pitch) || !std::isfinite(dt))
  {
    return {0.0, 0.0};
  }
  return {
    bounded_axis(
      requested_yaw_pitch[0], previous_yaw_pitch[0], limits.yaw, limits.yaw_rate, dt),
    bounded_axis(
      requested_yaw_pitch[1], previous_yaw_pitch[1], limits.pitch, limits.pitch_rate, dt)};
}

bool head_within_absolute_limits(
  const std::array<double, 2> & yaw_pitch,
  const HeadAbsoluteLimits & limits)
{
  const std::array<double, 4> bounds{
    limits.yaw_min, limits.yaw_max, limits.pitch_min, limits.pitch_max};
  if (!finite_values(yaw_pitch) ||
    !std::all_of(bounds.begin(), bounds.end(), [](double value) {
      return std::isfinite(value);
    }) ||
    limits.yaw_min > limits.yaw_max || limits.pitch_min > limits.pitch_max)
  {
    return false;
  }
  return yaw_pitch[0] >= limits.yaw_min && yaw_pitch[0] <= limits.yaw_max &&
         yaw_pitch[1] >= limits.pitch_min && yaw_pitch[1] <= limits.pitch_max;
}

std::array<double, 2> absolute_head_target(
  const std::array<double, 2> & origin_yaw_pitch,
  const std::array<double, 2> & relative_delta_yaw_pitch,
  const HeadAbsoluteLimits & limits)
{
  // An out-of-envelope seed must be rejected by the caller rather than
  // silently recentered.  NaN is an intentional fail-closed sentinel: the
  // existing finite-value checks reject it before a transport can use it.
  if (!head_within_absolute_limits(origin_yaw_pitch, limits) ||
    !finite_values(relative_delta_yaw_pitch))
  {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid};
  }
  const std::array<double, 2> candidate{
    origin_yaw_pitch[0] + relative_delta_yaw_pitch[0],
    origin_yaw_pitch[1] + relative_delta_yaw_pitch[1]};
  if (!finite_values(candidate)) {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid};
  }
  return {
    std::clamp(candidate[0], limits.yaw_min, limits.yaw_max),
    std::clamp(candidate[1], limits.pitch_min, limits.pitch_max)};
}

std::array<double, 2> step_head_toward_zero(
  const std::array<double, 2> & previous_yaw_pitch,
  double dt,
  double yaw_rate,
  double pitch_rate)
{
  if (!finite_values(previous_yaw_pitch) || !std::isfinite(dt) || dt < 0.0 ||
    !std::isfinite(yaw_rate) || yaw_rate <= 0.0 ||
    !std::isfinite(pitch_rate) || pitch_rate <= 0.0)
  {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid};
  }
  const double bounded_dt = std::clamp(dt, 0.0, 0.10);
  const auto step_axis = [bounded_dt](double value, double rate) {
      const double magnitude = std::abs(value);
      const double step = rate * bounded_dt;
      if (magnitude <= step) {
        return 0.0;
      }
      return std::copysign(magnitude - step, value);
    };
  return {
    step_axis(previous_yaw_pitch[0], yaw_rate),
    step_axis(previous_yaw_pitch[1], pitch_rate)};
}

}  // namespace r1_live_writer
