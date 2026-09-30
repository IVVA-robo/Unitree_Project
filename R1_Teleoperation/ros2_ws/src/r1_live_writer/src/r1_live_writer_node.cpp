#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "r1_live_writer/safety.hpp"
#include "r1_live_writer/sequential_control.hpp"
#include "r1_live_writer/transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace r1_live_writer
{

using SteadyClock = std::chrono::steady_clock;
using Trigger = std_srvs::srv::Trigger;

namespace
{

constexpr std::array<const char *, 13> kArmSdkJointNames = {
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
  "waist_yaw_joint", "head_pitch_joint", "head_yaw_joint"};

constexpr std::size_t kHeadPitchIndex = 11;
constexpr std::size_t kHeadYawIndex = 12;
constexpr std::size_t kWaistYawIndex = 10;
constexpr std::array<const char *, 12> kLegJointNames = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};
constexpr double kVelocity127ProbeMaxDurationSec = 1.5;
constexpr double kVelocity127ProbeMaxForwardMps = 0.15;
// LowState may report a few milliradians of servo overshoot at an official
// stop.  This margin is only for accepting feedback so the exhibition startup
// centering path can command the joint back inward.  Every published target
// remains clamped to the immutable manufacturer envelope below.
constexpr double kHeadFeedbackEnvelopeToleranceRad = 0.01;

bool head_within_feedback_envelope(
  const std::array<double, 2> & yaw_pitch,
  const HeadAbsoluteLimits & limits)
{
  if (!finite_values(yaw_pitch)) {
    return false;
  }
  return yaw_pitch[0] >= limits.yaw_min - kHeadFeedbackEnvelopeToleranceRad &&
         yaw_pitch[0] <= limits.yaw_max + kHeadFeedbackEnvelopeToleranceRad &&
         yaw_pitch[1] >= limits.pitch_min - kHeadFeedbackEnvelopeToleranceRad &&
         yaw_pitch[1] <= limits.pitch_max + kHeadFeedbackEnvelopeToleranceRad;
}

std::array<double, 2> clamp_head_to_absolute_envelope(
  const std::array<double, 2> & yaw_pitch,
  const HeadAbsoluteLimits & limits)
{
  return {
    std::clamp(yaw_pitch[0], limits.yaw_min, limits.yaw_max),
    std::clamp(yaw_pitch[1], limits.pitch_min, limits.pitch_max)};
}

enum class HeadRecenterState
{
  Disabled,
  Idle,
  PendingFreshSeed,
  ZeroWeightDiscovery,
  RampingWeight,
  ProbingOut,
  VerifyingOut,
  ReturningToSeed,
  ConfirmingReturn,
  // The public/manual full-recenter service remains blocked.  These two
  // states are used only by the bounded automatic exhibition startup path.
  LegacyMoving,
  LegacyConfirming,
};

const char * head_recenter_state_name(HeadRecenterState state)
{
  switch (state) {
    case HeadRecenterState::Disabled:
      return "disabled";
    case HeadRecenterState::Idle:
      return "idle";
    case HeadRecenterState::PendingFreshSeed:
      return "pending_fresh_seed";
    case HeadRecenterState::ZeroWeightDiscovery:
      return "probe_zero_weight_discovery";
    case HeadRecenterState::RampingWeight:
      return "probe_ramping_weight";
    case HeadRecenterState::ProbingOut:
      return "probe_out";
    case HeadRecenterState::VerifyingOut:
      return "probe_verifying_out";
    case HeadRecenterState::ReturningToSeed:
      return "probe_returning_to_seed";
    case HeadRecenterState::ConfirmingReturn:
      return "probe_confirming_return";
    case HeadRecenterState::LegacyMoving:
      return "exhibition_startup_centering";
    case HeadRecenterState::LegacyConfirming:
      return "exhibition_startup_center_confirming";
  }
  return "invalid";
}

bool finite(double value)
{
  return std::isfinite(value);
}

bool is_zero(const std::array<double, 3> & velocity)
{
  return std::all_of(velocity.begin(), velocity.end(), [](double value) {
    return std::abs(value) < 1.0e-6;
  });
}

std::string bool_text(bool value)
{
  return value ? "true" : "false";
}

constexpr char kRequiredPhysicalRmw[] = "rmw_fastrtps_cpp";

bool physical_rmw_is_fastdds()
{
  const char * const identifier = rmw_get_implementation_identifier();
  return identifier != nullptr && std::string(identifier) == kRequiredPhysicalRmw;
}

}  // namespace

class R1LiveWriter final : public rclcpp::Node
{
public:
  R1LiveWriter()
  : Node("r1_live_writer")
  {
    send_commands_ = declare_parameter<bool>("send_commands", false);
    enable_head_ = declare_parameter<bool>("enable_head", false);
    enable_arms_ = declare_parameter<bool>("enable_arms", false);
    enable_locomotion_ = declare_parameter<bool>("enable_locomotion", false);
    velocity_status_127_probe_enabled_ = declare_parameter<bool>(
      "velocity_status_127_probe_enabled", false);
    enable_prepare_ = declare_parameter<bool>("enable_prepare", false);
    prepare_enter_locomotion_ = declare_parameter<bool>(
      "prepare_enter_locomotion", false);
    prepare_speed_mode_ = declare_parameter<int>("prepare_speed_mode", -1);
    commissioning_confirmed_ = declare_parameter<bool>(
      "commissioning_confirmed", false);
    commissioning_token_ = declare_parameter<std::string>("commissioning_token", "");
    expected_vr_source_ip_ = declare_parameter<std::string>(
      "expected_vr_source_ip", "");
    vr_transport_ = declare_parameter<std::string>("vr_transport", "lan");
    transport_name_ = declare_parameter<std::string>("transport", "mock");
    arm_sdk_frame_profile_ = declare_parameter<std::string>("arm_sdk_frame_profile", "legacy");
    if (arm_sdk_frame_profile_ != "legacy" && arm_sdk_frame_profile_ != "a5_20260803") {
      throw std::runtime_error("arm_sdk_frame_profile must be legacy or a5_20260803");
    }
    locomotion_command_mode_ = declare_parameter<std::string>(
      "locomotion_command_mode", "loco_rpc");
    profile_ = declare_parameter<std::string>("profile", "slow-safe");
    network_interface_ = declare_parameter<std::string>(
      "network_interface", "enxb4b024be59fe");
    require_prepare_ = declare_parameter<bool>("require_prepare", true);
    static_prepare_mode_ = declare_parameter<bool>(
      "static_prepare_mode", false);
    exhibition_session_mode_ = declare_parameter<bool>(
      "exhibition_session_mode", false);
    sequential_control_enabled_ = declare_parameter<bool>("sequential_control", false);
    if (sequential_control_enabled_ &&
      (!exhibition_session_mode_ || !enable_head_ || !enable_arms_ ||
      !enable_locomotion_ || !require_prepare_ || !prepare_enter_locomotion_ ||
      locomotion_command_mode_ != "wireless_controller"))
    {
      throw std::runtime_error(
        "sequential_control requires prepared exhibition full wireless-controller mode");
    }
    exhibition_session_armed_on_start_ = declare_parameter<bool>(
      "exhibition_session_armed_on_start", false);
    exhibition_reconnect_grace_sec_ = declare_parameter<double>(
      "exhibition_reconnect_grace_sec", 2.0);

    head_topic_ = declare_parameter<std::string>(
      "head_topic", "/r1_hardware_adapter/debug/head/joint_trajectory");
    arm_topic_ = declare_parameter<std::string>(
      "arm_topic", "/r1_kinematics_control/debug/arm_trajectory");
    velocity_topic_ = declare_parameter<std::string>(
      "velocity_topic", "/r1/locomotion_dry_run/debug/cmd_vel");
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/r1/sdk/joint_states");
    motor_health_topic_ = declare_parameter<std::string>(
      "motor_health_topic", "/r1/sdk_transport/motors_healthy");
    deadman_topic_ = declare_parameter<std::string>(
      "deadman_topic", "/vr/teleop/active");
    session_armed_topic_ = declare_parameter<std::string>(
      "session_armed_topic", "/vr/teleop/session_armed");
    kill_topic_ = declare_parameter<std::string>(
      "kill_topic", "/r1/safety/kill");
    kill_request_topic_ = declare_parameter<std::string>(
      "kill_request_topic", "/r1/safety/kill_request");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/r1/live_writer/status");
    debug_head_topic_ = declare_parameter<std::string>(
      "debug_head_topic", "/r1/live_writer/debug/head_command");
    debug_arm_topic_ = declare_parameter<std::string>(
      "debug_arm_topic", "/r1/live_writer/debug/arm_command");
    debug_velocity_topic_ = declare_parameter<std::string>(
      "debug_velocity_topic", "/r1/live_writer/debug/cmd_vel");

    state_timeout_sec_ = declare_parameter<double>("state_timeout_sec", 0.50);
    motor_health_timeout_sec_ = declare_parameter<double>(
      "motor_health_timeout_sec", 0.50);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.25);
    deadman_timeout_sec_ = declare_parameter<double>("deadman_timeout_sec", 1.50);
    kill_timeout_sec_ = declare_parameter<double>("kill_timeout_sec", 0.50);
    // Unitree's R1 ArmSdk routine requires a new rt/arm_sdk frame every
    // 10 ms.  The publisher does not repeat queued frames on its own, so the
    // writer timer itself must run at 100 Hz whenever physical arm control is
    // enabled.
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    command_duration_sec_ = declare_parameter<double>("command_duration_sec", 1.0);
    // ArmSdk needs the 100 Hz timer, but the R1 locomotion interface is an
    // RPC whose command already remains valid for one second.  Keep that RPC
    // on its own bounded refresh rate instead of flooding ai_sport from the
    // arm publication loop.
    locomotion_rpc_rate_hz_ = declare_parameter<double>(
      "locomotion_rpc_rate_hz", 10.0);
    // Unitree Explore publishes its virtual joystick on
    // rt/wirelesscontroller at approximately 20 Hz. Keep that cadence
    // separate from the legacy LocoClient RPC refresh rate.
    wireless_controller_rate_hz_ = declare_parameter<double>(
      "wireless_controller_rate_hz", 20.0);

    velocity_limits_.forward = declare_parameter<double>(
      "max_forward_mps", velocity_limits_.forward);
    velocity_limits_.lateral = declare_parameter<double>(
      "max_lateral_mps", velocity_limits_.lateral);
    velocity_limits_.yaw = declare_parameter<double>(
      "max_yaw_rps", velocity_limits_.yaw);
    velocity_limits_.linear_rate = declare_parameter<double>(
      "max_linear_rate_mps2", velocity_limits_.linear_rate);
    velocity_limits_.yaw_rate = declare_parameter<double>(
      "max_yaw_rate_rps2", velocity_limits_.yaw_rate);
    head_limits_.yaw = declare_parameter<double>(
      "max_head_yaw_delta_rad", head_limits_.yaw);
    head_limits_.pitch = declare_parameter<double>(
      "max_head_pitch_delta_rad", head_limits_.pitch);
    head_limits_.yaw_rate = declare_parameter<double>(
      "max_head_yaw_delta_rate_rad_s", head_limits_.yaw_rate);
    head_limits_.pitch_rate = declare_parameter<double>(
      "max_head_pitch_delta_rate_rad_s", head_limits_.pitch_rate);
    arm_joint_delta_limit_rad_ = declare_parameter<double>(
      "max_arm_joint_delta_rad", arm_joint_delta_limit_rad_);
    arm_joint_rate_rad_s_ = declare_parameter<double>(
      "max_arm_joint_rate_rad_s", arm_joint_rate_rad_s_);
    arm_targets_absolute_ = declare_parameter<bool>("arm_targets_absolute", false);
    arm_feedback_follow_command_delta_rad_ = declare_parameter<double>(
      "arm_feedback_follow_command_delta_rad", arm_feedback_follow_command_delta_rad_);
    arm_feedback_follow_min_progress_rad_ = declare_parameter<double>(
      "arm_feedback_follow_min_progress_rad", arm_feedback_follow_min_progress_rad_);
    arm_feedback_follow_timeout_sec_ = declare_parameter<double>(
      "arm_feedback_follow_timeout_sec", arm_feedback_follow_timeout_sec_);
    head_absolute_envelope_confirmed_ = declare_parameter<bool>(
      "head_absolute_envelope_confirmed", false);
    max_head_tracking_seed_yaw_rad_ = declare_parameter<double>(
      "max_head_tracking_seed_yaw_rad", 0.35);
    max_head_tracking_seed_pitch_rad_ = declare_parameter<double>(
      "max_head_tracking_seed_pitch_rad", 0.25);
    max_head_seed_velocity_rad_s_ = declare_parameter<double>(
      "max_head_seed_velocity_rad_s", 0.05);
    head_recenter_enabled_ = declare_parameter<bool>("head_recenter_enabled", false);
    head_recenter_confirmed_ = declare_parameter<bool>("head_recenter_confirmed", false);
    head_recenter_yaw_rate_rad_s_ = declare_parameter<double>(
      "head_recenter_yaw_rate_rad_s", 0.08);
    head_recenter_pitch_rate_rad_s_ = declare_parameter<double>(
      "head_recenter_pitch_rate_rad_s", 0.05);
    head_recenter_max_feedback_velocity_rad_s_ = declare_parameter<double>(
      "head_recenter_max_feedback_velocity_rad_s", 0.12);
    head_recenter_seed_stability_rad_ = declare_parameter<double>(
      "head_recenter_seed_stability_rad", 0.01);
    head_recenter_tolerance_rad_ = declare_parameter<double>(
      "head_recenter_tolerance_rad", 0.02);
    head_recenter_follow_tolerance_rad_ = declare_parameter<double>(
      "head_recenter_follow_tolerance_rad", 0.02);
    head_recenter_follow_timeout_sec_ = declare_parameter<double>(
      "head_recenter_follow_timeout_sec", 1.0);
    head_recenter_timeout_sec_ = declare_parameter<double>(
      "head_recenter_timeout_sec", 35.0);
    head_ownership_probe_only_ = declare_parameter<bool>(
      "head_ownership_probe_only", true);
    head_ownership_probe_confirmed_ = declare_parameter<bool>(
      "head_ownership_probe_confirmed", false);
    head_probe_zero_weight_hold_sec_ = declare_parameter<double>(
      "head_probe_zero_weight_hold_sec", 0.35);
    head_probe_weight_ramp_sec_ = declare_parameter<double>(
      "head_probe_weight_ramp_sec", 1.0);
    head_probe_step_rad_ = declare_parameter<double>(
      "head_probe_step_rad", 0.005);
    head_probe_rate_rad_s_ = declare_parameter<double>(
      "head_probe_rate_rad_s", 0.005);
    head_probe_max_feedback_velocity_rad_s_ = declare_parameter<double>(
      "head_probe_max_feedback_velocity_rad_s", 0.04);
    head_probe_min_progress_rad_ = declare_parameter<double>(
      "head_probe_min_progress_rad", 0.0015);
    head_probe_seed_stability_rad_ = declare_parameter<double>(
      "head_probe_seed_stability_rad", 0.003);
    head_probe_other_joint_tolerance_rad_ = declare_parameter<double>(
      "head_probe_other_joint_tolerance_rad", 0.003);
    head_probe_ramp_other_joint_tolerance_rad_ = declare_parameter<double>(
      "head_probe_ramp_other_joint_tolerance_rad", 0.015);
    head_probe_claim_tolerance_rad_ = declare_parameter<double>(
      "head_probe_claim_tolerance_rad", 0.003);
    head_probe_direction_tolerance_rad_ = declare_parameter<double>(
      "head_probe_direction_tolerance_rad", 0.0005);
    head_probe_return_tolerance_rad_ = declare_parameter<double>(
      "head_probe_return_tolerance_rad", 0.0015);
    head_probe_follow_timeout_sec_ = declare_parameter<double>(
      "head_probe_follow_timeout_sec", 1.5);
    head_probe_total_timeout_sec_ = declare_parameter<double>(
      "head_probe_total_timeout_sec", 8.0);
    head_probe_required_feedback_samples_ = declare_parameter<int>(
      "head_probe_required_feedback_samples", 3);
    head_probe_seed_samples_ = declare_parameter<int>(
      "head_probe_seed_samples", 10);

    validate_parameters();
    arm_response_guard_ = ArmResponseGuard(
      arm_feedback_follow_command_delta_rad_, arm_feedback_follow_min_progress_rad_,
      arm_feedback_follow_timeout_sec_);
    session_control_seen_ = exhibition_session_armed_on_start_;
    session_control_armed_ = exhibition_session_armed_on_start_;
    head_recenter_state_ = head_recenter_enabled_ ?
      HeadRecenterState::Idle : HeadRecenterState::Disabled;

    if (transport_name_ == "sdk" && !physical_rmw_is_fastdds()) {
      const char * const identifier = rmw_get_implementation_identifier();
      throw std::runtime_error(
              "SDK transport requires active RMW " +
              std::string(kRequiredPhysicalRmw) + "; actual=" +
              std::string(identifier == nullptr ? "unavailable" : identifier));
    }

    if (transport_name_ == "mock") {
      transport_ = std::make_unique<MockTransport>();
    } else if (transport_name_ == "sdk") {
      transport_ = std::make_unique<SdkTransport>(
        locomotion_command_mode_ == "wireless_controller",
        arm_sdk_frame_profile_ == "a5_20260803");
    } else {
      throw std::runtime_error("transport must be either mock or sdk");
    }

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto latch_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, latch_qos);
    armed_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/r1/live_writer/armed", latch_qos);
    kill_request_publisher_ = create_publisher<std_msgs::msg::Bool>(
      kill_request_topic_, latch_qos);
    debug_head_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      debug_head_topic_, reliable_qos);
    debug_arm_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      debug_arm_topic_, reliable_qos);
    debug_velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      debug_velocity_topic_, reliable_qos);

    head_subscription_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      head_topic_, reliable_qos,
      std::bind(&R1LiveWriter::on_head, this, std::placeholders::_1));
    arm_subscription_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      arm_topic_, reliable_qos,
      std::bind(&R1LiveWriter::on_arm, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      velocity_topic_, reliable_qos,
      std::bind(&R1LiveWriter::on_velocity, this, std::placeholders::_1));
    state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, reliable_qos,
      std::bind(&R1LiveWriter::on_joint_state, this, std::placeholders::_1));
    if (motor_health_required()) {
      motor_health_subscription_ = create_subscription<std_msgs::msg::Bool>(
        motor_health_topic_, reliable_qos,
        std::bind(&R1LiveWriter::on_motor_health, this, std::placeholders::_1));
    }
    deadman_subscription_ = create_subscription<std_msgs::msg::Bool>(
      deadman_topic_, reliable_qos,
      std::bind(&R1LiveWriter::on_deadman, this, std::placeholders::_1));
    session_armed_subscription_ = create_subscription<std_msgs::msg::Bool>(
      session_armed_topic_, latch_qos,
      std::bind(&R1LiveWriter::on_session_armed, this, std::placeholders::_1));
    kill_subscription_ = create_subscription<std_msgs::msg::Bool>(
      kill_topic_, latch_qos,
      std::bind(&R1LiveWriter::on_kill, this, std::placeholders::_1));

    prepare_service_ = create_service<Trigger>(
      "/r1/live_writer/prepare",
      std::bind(
        &R1LiveWriter::on_prepare, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<Trigger>(
      "/r1/live_writer/stop",
      std::bind(
        &R1LiveWriter::on_stop, this, std::placeholders::_1, std::placeholders::_2));
    kill_service_ = create_service<Trigger>(
      "/r1/live_writer/kill",
      std::bind(
        &R1LiveWriter::on_kill_service, this,
        std::placeholders::_1, std::placeholders::_2));
    recenter_head_service_ = create_service<Trigger>(
      "/r1/live_writer/recenter_head",
      std::bind(
        &R1LiveWriter::on_recenter_head, this,
        std::placeholders::_1, std::placeholders::_2));
    probe_head_ownership_service_ = create_service<Trigger>(
      "/r1/live_writer/probe_head_ownership",
      std::bind(
        &R1LiveWriter::on_probe_head_ownership, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_kill_service_ = create_service<Trigger>(
      "/r1/live_writer/reset_kill",
      std::bind(
        &R1LiveWriter::on_reset_kill, this,
        std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&R1LiveWriter::on_timer, this));

    publish_armed();
    publish_status(
      "startup fail_closed=true transport=" + transport_name_ +
      " arm_sdk_frame_profile=" + arm_sdk_frame_profile_ +
      " locomotion_command_mode=" + locomotion_command_mode_ +
      " send_commands=" + bool_text(send_commands_) +
      " enable_head=" + bool_text(enable_head_) +
      " enable_arms=" + bool_text(enable_arms_) +
      " absolute_head_envelope_confirmed=" +
      bool_text(head_absolute_envelope_confirmed_) +
      " head_recenter_enabled=" + bool_text(head_recenter_enabled_) +
      " head_recenter_confirmed=" + bool_text(head_recenter_confirmed_) +
      " head_ownership_probe_only=" + bool_text(head_ownership_probe_only_) +
      " head_ownership_probe_confirmed=" +
      bool_text(head_ownership_probe_confirmed_) +
      " exhibition_auto_head_center=" +
      bool_text(exhibition_session_mode_ && enable_head_ && !head_recenter_enabled_) +
      " enable_locomotion=" + bool_text(enable_locomotion_) +
      " velocity_status_127_probe=" + bool_text(velocity_status_127_probe_enabled_) +
      " enable_prepare=" + bool_text(enable_prepare_) +
      " prepare_enter_locomotion=" + bool_text(prepare_enter_locomotion_) +
      " prepare_speed_mode=" + std::to_string(prepare_speed_mode_) +
      " static_prepare_mode=" + bool_text(static_prepare_mode_) +
      " exhibition_session_mode=" + bool_text(exhibition_session_mode_) +
      " session_armed=" + bool_text(session_control_armed_) +
      " motor_health_required=" + bool_text(motor_health_required()) +
      " publish_rate_hz=" + std::to_string(publish_rate_hz_) +
      " locomotion_rpc_rate_hz=" + std::to_string(locomotion_rpc_rate_hz_) +
      " wireless_controller_rate_hz=" +
      std::to_string(wireless_controller_rate_hz_) +
      " profile=" + profile_ +
      " waiting_for=fresh_kill_clear,deadman,state,motor_health,command",
      true);
    RCLCPP_WARN(
      get_logger(),
      "R1 live writer starts fail-closed. SDK construction and every send are gated; "
      "current transport=%s send_commands=%s",
      transport_name_.c_str(), bool_text(send_commands_).c_str());
  }

  ~R1LiveWriter() override
  {
    prepare_cancel_requested_.store(true);
    if (prepare_future_.valid()) {
      prepare_future_.wait();
      try {
        (void)prepare_future_.get();
      } catch (...) {
        // Shutdown must continue to the transport's bounded stop/release path.
      }
    }
    prepare_in_progress_ = false;
    safe_stop_outputs("node_shutdown", false);
  }

private:
  void validate_parameters() const
  {
    const std::array<double, 11> positive = {
      state_timeout_sec_, motor_health_timeout_sec_, command_timeout_sec_,
      deadman_timeout_sec_,
      kill_timeout_sec_, publish_rate_hz_, command_duration_sec_,
      velocity_limits_.forward, velocity_limits_.lateral,
      velocity_limits_.yaw, velocity_limits_.linear_rate};
    if (!std::all_of(positive.begin(), positive.end(), [](double value) {
        return finite(value) && value > 0.0;
      }) || !finite(velocity_limits_.yaw_rate) || velocity_limits_.yaw_rate <= 0.0 ||
      !finite(head_limits_.yaw) || head_limits_.yaw <= 0.0 ||
      !finite(head_limits_.pitch) || head_limits_.pitch <= 0.0 ||
      !finite(head_limits_.yaw_rate) || head_limits_.yaw_rate <= 0.0 ||
      !finite(head_limits_.pitch_rate) || head_limits_.pitch_rate <= 0.0)
    {
      throw std::runtime_error("all timeouts, rates, and limits must be finite and positive");
    }
    if (!finite(exhibition_reconnect_grace_sec_) ||
      exhibition_reconnect_grace_sec_ < 1.0 || exhibition_reconnect_grace_sec_ > 3.0)
    {
      throw std::runtime_error(
              "exhibition_reconnect_grace_sec must be finite and in [1, 3]");
    }
    if (exhibition_session_armed_on_start_ && !exhibition_session_mode_) {
      throw std::runtime_error(
              "exhibition_session_armed_on_start requires exhibition_session_mode");
    }
    if (!finite(arm_joint_delta_limit_rad_) || arm_joint_delta_limit_rad_ <= 0.0 ||
      !finite(arm_joint_rate_rad_s_) || arm_joint_rate_rad_s_ <= 0.0 ||
      arm_joint_delta_limit_rad_ > 0.25 || arm_joint_rate_rad_s_ > 1.0)
    {
      throw std::runtime_error(
              "arm joint slew limits exceed the immutable first-live ceiling");
    }
    const std::array<double, 3> arm_feedback_follow_values = {
      arm_feedback_follow_command_delta_rad_,
      arm_feedback_follow_min_progress_rad_,
      arm_feedback_follow_timeout_sec_};
    if (!std::all_of(
        arm_feedback_follow_values.begin(), arm_feedback_follow_values.end(), finite) ||
      arm_feedback_follow_command_delta_rad_ < 0.05 ||
      arm_feedback_follow_command_delta_rad_ > 0.15 ||
      arm_feedback_follow_min_progress_rad_ < 0.001 ||
      arm_feedback_follow_min_progress_rad_ > arm_feedback_follow_command_delta_rad_ ||
      arm_feedback_follow_timeout_sec_ <= 0.0 ||
      arm_feedback_follow_timeout_sec_ > 1.50)
    {
      throw std::runtime_error(
              "arm feedback follow guard exceeds immutable first-live ceilings");
    }
    if (state_timeout_sec_ > 0.50 || motor_health_timeout_sec_ > 0.50 ||
      command_timeout_sec_ > 0.25 ||
      deadman_timeout_sec_ > 2.0 || kill_timeout_sec_ > 1.0)
    {
      throw std::runtime_error("a configured freshness timeout exceeds the live ceiling");
    }
    if (velocity_limits_.forward > 0.20 || velocity_limits_.lateral > 0.12 ||
      velocity_limits_.yaw > 0.35 || velocity_limits_.linear_rate > 0.25 ||
      velocity_limits_.yaw_rate > 0.50)
    {
      throw std::runtime_error("locomotion limits exceed the immutable first-live ceiling");
    }
    if (velocity_status_127_probe_enabled_ &&
      (transport_name_ != "sdk" || !send_commands_ || !enable_locomotion_ ||
      enable_head_ || enable_arms_ || !enable_prepare_ ||
      !prepare_enter_locomotion_ || exhibition_session_mode_ ||
      velocity_limits_.forward > kVelocity127ProbeMaxForwardMps ||
      !velocity_127_probe_environment_confirmed()))
    {
      throw std::runtime_error(
              "velocity status-127 probe requires SDK legs-only mode, "
              "forward limit <=0.15 m/s, ordinary Deadman, FSM 811 prepare, "
              "and all dedicated environment confirmations");
    }
    // Exhibition may track faster inside the same position envelope. Keep
    // first-live rates for commissioning and cap both profiles explicitly.
    const double head_yaw_rate_ceiling = exhibition_session_mode_ ? 1.0 : 0.35;
    const double head_pitch_rate_ceiling = exhibition_session_mode_ ? 1.0 : 0.25;
    if (head_limits_.yaw > 0.35 || head_limits_.pitch > 0.25 ||
      head_limits_.yaw_rate > head_yaw_rate_ceiling ||
      head_limits_.pitch_rate > head_pitch_rate_ceiling)
    {
      throw std::runtime_error(
              "headset-relative head limits exceed the bounded response profile");
    }
    const std::array<double, 3> head_seed_values = {
      max_head_tracking_seed_yaw_rad_, max_head_tracking_seed_pitch_rad_,
      max_head_seed_velocity_rad_s_};
    if (!std::all_of(head_seed_values.begin(), head_seed_values.end(), finite) ||
      max_head_tracking_seed_yaw_rad_ <= 0.0 ||
      max_head_tracking_seed_pitch_rad_ <= 0.0 ||
      max_head_seed_velocity_rad_s_ <= 0.0)
    {
      throw std::runtime_error(
              "head tracking seed limits must be finite and positive");
    }
    if (max_head_tracking_seed_yaw_rad_ > 0.35 ||
      max_head_tracking_seed_pitch_rad_ > 0.25 ||
      max_head_seed_velocity_rad_s_ > 0.10)
    {
      throw std::runtime_error(
              "head tracking seed window exceeds the immutable first-live ceiling");
    }
    const std::array<double, 8> recenter_values = {
      head_recenter_yaw_rate_rad_s_, head_recenter_pitch_rate_rad_s_,
      head_recenter_max_feedback_velocity_rad_s_,
      head_recenter_seed_stability_rad_,
      head_recenter_tolerance_rad_, head_recenter_follow_tolerance_rad_,
      head_recenter_follow_timeout_sec_, head_recenter_timeout_sec_};
    if (!std::all_of(recenter_values.begin(), recenter_values.end(), [](double value) {
        return finite(value) && value > 0.0;
      }))
    {
      throw std::runtime_error("head recenter limits must be finite and positive");
    }
    if (head_recenter_yaw_rate_rad_s_ > 0.08 ||
      head_recenter_pitch_rate_rad_s_ > 0.05 ||
      head_recenter_max_feedback_velocity_rad_s_ > 0.15 ||
      head_recenter_seed_stability_rad_ > 0.01 ||
      head_recenter_tolerance_rad_ > 0.02 ||
      head_recenter_follow_tolerance_rad_ > 0.02 ||
      head_recenter_follow_timeout_sec_ > 1.0 ||
      head_recenter_timeout_sec_ > 40.0)
    {
      throw std::runtime_error("head recenter limits exceed immutable safety ceilings");
    }
    const std::array<double, 14> probe_values = {
      head_probe_zero_weight_hold_sec_, head_probe_weight_ramp_sec_,
      head_probe_step_rad_, head_probe_rate_rad_s_,
      head_probe_max_feedback_velocity_rad_s_,
      head_probe_min_progress_rad_, head_probe_seed_stability_rad_,
      head_probe_other_joint_tolerance_rad_,
      head_probe_ramp_other_joint_tolerance_rad_, head_probe_claim_tolerance_rad_,
      head_probe_direction_tolerance_rad_, head_probe_return_tolerance_rad_,
      head_probe_follow_timeout_sec_, head_probe_total_timeout_sec_};
    if (!std::all_of(probe_values.begin(), probe_values.end(), [](double value) {
        return finite(value) && value > 0.0;
      }))
    {
      throw std::runtime_error("head ownership probe limits must be finite and positive");
    }
    if (head_probe_zero_weight_hold_sec_ < 0.25 ||
      head_probe_zero_weight_hold_sec_ > 0.50 ||
      head_probe_weight_ramp_sec_ < 0.80 ||
      head_probe_weight_ramp_sec_ > 1.20 ||
      head_probe_step_rad_ > 0.005 ||
      head_probe_rate_rad_s_ > 0.005 ||
      head_probe_max_feedback_velocity_rad_s_ > 0.04 ||
      head_probe_min_progress_rad_ > head_probe_step_rad_ ||
      head_probe_min_progress_rad_ > 0.0025 ||
      head_probe_seed_stability_rad_ > 0.003 ||
      head_probe_other_joint_tolerance_rad_ > 0.003 ||
      head_probe_ramp_other_joint_tolerance_rad_ <
      head_probe_other_joint_tolerance_rad_ ||
      head_probe_ramp_other_joint_tolerance_rad_ > 0.02 ||
      head_probe_claim_tolerance_rad_ > 0.003 ||
      head_probe_direction_tolerance_rad_ > 0.001 ||
      head_probe_return_tolerance_rad_ > 0.002 ||
      head_probe_follow_timeout_sec_ > 2.0 ||
      head_probe_total_timeout_sec_ < 4.0 ||
      head_probe_total_timeout_sec_ > 10.0 ||
      head_probe_required_feedback_samples_ < 3 ||
      head_probe_required_feedback_samples_ > 5 ||
      head_probe_seed_samples_ < 10 ||
      head_probe_seed_samples_ > 20)
    {
      throw std::runtime_error(
              "head ownership probe settings exceed immutable micro-probe ceilings");
    }
    if (head_recenter_enabled_ &&
      (!enable_head_ || enable_arms_ || enable_locomotion_ ||
      !enable_prepare_ || !require_prepare_))
    {
      throw std::runtime_error(
              "head recenter requires head-only mode with mandatory prepare");
    }
    if (transport_name_ == "sdk" && send_commands_ && enable_head_ &&
      !head_absolute_envelope_confirmed_)
    {
      throw std::runtime_error(
              "physical head mode requires head_absolute_envelope_confirmed=true");
    }
    if (transport_name_ == "sdk" && send_commands_ && head_recenter_enabled_ &&
      !head_ownership_probe_only_)
    {
      throw std::runtime_error(
              "physical head recenter is suspended; SDK mode requires "
              "head_ownership_probe_only=true");
    }
    if (transport_name_ == "sdk" && send_commands_ && head_recenter_enabled_ &&
      (!head_ownership_probe_confirmed_ ||
      !head_ownership_probe_environment_confirmed()))
    {
      throw std::runtime_error(
              "physical head ownership probe requires "
              "head_ownership_probe_confirmed=true and "
              "ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE=1; the legacy recenter "
              "acknowledgement is intentionally insufficient");
    }
    if (std::abs(command_duration_sec_ - 1.0) > 1.0e-6) {
      throw std::runtime_error(
              "command_duration_sec must be 1.0 for the R1 sport firmware");
    }
    if (!finite(locomotion_rpc_rate_hz_) || locomotion_rpc_rate_hz_ < 2.0 ||
      locomotion_rpc_rate_hz_ > 10.0)
    {
      throw std::runtime_error(
              "locomotion_rpc_rate_hz must be between 2 and 10 Hz");
    }
    if (!finite(wireless_controller_rate_hz_) ||
      wireless_controller_rate_hz_ < 15.0 || wireless_controller_rate_hz_ > 25.0)
    {
      throw std::runtime_error(
              "wireless_controller_rate_hz must be between 15 and 25 Hz");
    }
    if (locomotion_command_mode_ != "loco_rpc" &&
      locomotion_command_mode_ != "wireless_controller")
    {
      throw std::runtime_error(
              "locomotion_command_mode must be loco_rpc or wireless_controller");
    }
    if (velocity_status_127_probe_enabled_ &&
      locomotion_command_mode_ != "loco_rpc")
    {
      throw std::runtime_error(
              "velocity status-127 probe requires locomotion_command_mode=loco_rpc");
    }
    if (publish_rate_hz_ < 10.0 || publish_rate_hz_ > 100.0) {
      throw std::runtime_error("publish_rate_hz must be between 10 and 100 Hz");
    }
    if (transport_name_ == "sdk" && send_commands_ && (enable_head_ || enable_arms_) &&
      std::abs(publish_rate_hz_ - 100.0) > 1.0e-6)
    {
      throw std::runtime_error(
              "physical R1 ArmSdk control requires publish_rate_hz=100 Hz");
    }
    const std::array<std::string, 13> topics = {
      head_topic_, arm_topic_, velocity_topic_, joint_state_topic_, motor_health_topic_,
      deadman_topic_, kill_topic_,
      kill_request_topic_, status_topic_, debug_head_topic_, debug_arm_topic_,
      debug_velocity_topic_,
      network_interface_};
    if (std::any_of(topics.begin(), topics.end(), [](const std::string & value) {
        return value.empty();
      }))
    {
      throw std::runtime_error("topic names and network_interface must not be empty");
    }
    for (const auto & topic : {
        head_topic_, arm_topic_, velocity_topic_, joint_state_topic_, motor_health_topic_,
        deadman_topic_, kill_topic_,
        kill_request_topic_, status_topic_, debug_head_topic_, debug_arm_topic_,
        debug_velocity_topic_})
    {
      if (topic.front() != '/') {
        throw std::runtime_error("all ROS topic parameters must be absolute names");
      }
    }
    if (send_commands_ && !static_prepare_mode_ &&
      !valid_vr_source(expected_vr_source_ip_, vr_transport_))
    {
      throw std::runtime_error(
              "send_commands=true requires a LAN source or explicit USB loopback source");
    }
    if (send_commands_ && (!enable_prepare_ || !require_prepare_)) {
      throw std::runtime_error(
              "send_commands=true requires enable_prepare=true and require_prepare=true");
    }
    if (prepare_enter_locomotion_ && !enable_prepare_) {
      throw std::runtime_error(
              "prepare_enter_locomotion=true requires enable_prepare=true");
    }
    if (prepare_speed_mode_ != -1 && prepare_speed_mode_ != 1) {
      throw std::runtime_error("prepare_speed_mode must be -1 (disabled) or 1");
    }
    if (prepare_speed_mode_ >= 0 &&
      (!prepare_enter_locomotion_ || !enable_locomotion_ ||
      !velocity_status_127_probe_enabled_))
    {
      throw std::runtime_error(
              "prepare_speed_mode is admitted only by the isolated legs-only status-127 probe");
    }
    if (static_prepare_mode_ &&
      (transport_name_ != "sdk" || !send_commands_ || !enable_prepare_ ||
      !require_prepare_ || enable_head_ || enable_arms_ ||
      enable_locomotion_ || prepare_enter_locomotion_ ||
      exhibition_session_mode_))
    {
      throw std::runtime_error(
              "static_prepare_mode requires SDK/send_commands/prepare only; "
              "head, arms, locomotion, FSM 811, and exhibition session must be disabled");
    }
    if (prepare_enter_locomotion_ && !enable_locomotion_) {
      const bool arms_only = enable_arms_ && !enable_head_;
      const bool head_only = enable_head_ && !enable_arms_;
      if (!arms_only && !head_only) {
        throw std::runtime_error(
                "velocity-disabled FSM 811 requires exactly one upper-body commissioning path");
      }
    }
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (!message || message->name.size() != message->position.size() ||
      message->name.size() != message->velocity.size())
    {
      state_valid_ = false;
      head_velocity_valid_ = false;
      publish_status("joint_state_rejected=name_position_velocity_count", true);
      return;
    }
    std::map<std::string, double> positions;
    std::map<std::string, double> velocities;
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      if (!finite(message->position[index]) || !finite(message->velocity[index]) ||
        !positions.emplace(message->name[index], message->position[index]).second ||
        !velocities.emplace(message->name[index], message->velocity[index]).second)
      {
        state_valid_ = false;
        head_velocity_valid_ = false;
        publish_status("joint_state_rejected=nonfinite_or_duplicate", true);
        return;
      }
    }
    ArmSdkPositions candidate{};
    for (std::size_t index = 0; index < kArmSdkJointNames.size(); ++index) {
      const auto found = positions.find(kArmSdkJointNames[index]);
      if (found == positions.end()) {
        state_valid_ = false;
        head_velocity_valid_ = false;
        publish_status(
          "joint_state_rejected=missing:" + std::string(kArmSdkJointNames[index]), true);
        return;
      }
      candidate[index] = found->second;
    }
    if (!sane_arm_sdk_feedback(candidate)) {
      state_valid_ = false;
      head_velocity_valid_ = false;
      publish_status("joint_state_rejected=outside_arm_sdk_sanity_envelope", true);
      return;
    }
    const std::array<double, 2> candidate_head{
      candidate[kHeadYawIndex], candidate[kHeadPitchIndex]};
    // LOCK/static prepare owns only the official Loco FSM client.  It never
    // constructs an ArmSdk writer and cannot command the head, so an existing
    // firmware-selected head pose must not block SetFsmId(4).  Full RUN/VR
    // sessions still require the immutable command envelope before any upper
    // body writer can be authorized.
    // StandUp/Start own the head while prepare is in progress. A transient
    // firmware-selected pose must not be mistaken for an ArmSdk command.
    // No upper-body publication is possible in that phase. Normal/recenter
    // seed paths recheck the absolute envelope before acquiring ownership.
    if (!static_prepare_mode_ && !prepare_in_progress_ &&
      !head_within_feedback_envelope(candidate_head, head_absolute_limits_))
    {
      state_valid_ = false;
      head_velocity_valid_ = false;
      std::ostringstream detail;
      detail << "joint_state_rejected=head_outside_official_absolute_envelope_plus_feedback_margin"
             << " yaw=" << candidate_head[0]
             << " pitch=" << candidate_head[1]
             << " yaw_min=" << head_absolute_limits_.yaw_min
             << " yaw_max=" << head_absolute_limits_.yaw_max
             << " pitch_min=" << head_absolute_limits_.pitch_min
             << " pitch_max=" << head_absolute_limits_.pitch_max
             << " feedback_margin=" << kHeadFeedbackEnvelopeToleranceRad;
      publish_status(detail.str(), true);
      return;
    }
    latest_head_velocity_ = {
      velocities.at("head_yaw_joint"), velocities.at("head_pitch_joint")};
    head_velocity_valid_ = true;
    legs_quiet_ = true;
    for (const auto * name : kLegJointNames) {
      const auto found = velocities.find(name);
      if (found == velocities.end() || std::abs(found->second) > 0.10) {
        legs_quiet_ = false;
      }
    }
    latest_state_ = candidate;
    state_valid_ = true;
    state_arrival_ = SteadyClock::now();
    ++state_sequence_;
  }

  void on_motor_health(const std_msgs::msg::Bool::SharedPtr message)
  {
    motor_health_seen_ = static_cast<bool>(message);
    motors_healthy_ = message && message->data;
    motor_health_arrival_ = SteadyClock::now();
    // A fresh false is an explicit firmware/motor fault report. During any
    // operation that can own an SDK output, stop immediately and latch kill;
    // while idle it remains a hard authorization block without opening SDK.
    if (motor_health_required() && !motors_healthy_ &&
      (outputs_active() || prepare_in_progress_ || head_recenter_requested()))
    {
      safe_stop_outputs("motor_health_reported_unhealthy", true);
    }
  }

  void on_head(const trajectory_msgs::msg::JointTrajectory::SharedPtr message)
  {
    if (!message || message->points.empty()) {
      head_valid_ = false;
      publish_status("head_rejected=empty", true);
      return;
    }
    const auto & point = message->points.front();
    if (message->joint_names.size() != point.positions.size()) {
      head_valid_ = false;
      publish_status("head_rejected=name_position_count", true);
      return;
    }
    bool have_yaw = false;
    bool have_pitch = false;
    std::array<double, 2> candidate{};
    for (std::size_t index = 0; index < message->joint_names.size(); ++index) {
      const double value = point.positions[index];
      if (!finite(value)) {
        head_valid_ = false;
        publish_status("head_rejected=nonfinite", true);
        return;
      }
      if (message->joint_names[index] == "head_yaw_joint" && !have_yaw) {
        candidate[0] = value;
        have_yaw = true;
      } else if (message->joint_names[index] == "head_pitch_joint" && !have_pitch) {
        candidate[1] = value;
        have_pitch = true;
      } else {
        head_valid_ = false;
        publish_status("head_rejected=unknown_or_duplicate_joint", true);
        return;
      }
    }
    if (!have_yaw || !have_pitch) {
      head_valid_ = false;
      publish_status("head_rejected=both_yaw_and_pitch_required", true);
      return;
    }
    latest_head_ = candidate;
    head_valid_ = true;
    head_arrival_ = SteadyClock::now();
    ++head_sequence_;
  }

  void on_arm(const trajectory_msgs::msg::JointTrajectory::SharedPtr message)
  {
    if (!message || message->points.empty()) {
      arm_valid_ = false;
      publish_status("arm_rejected=empty", true);
      return;
    }
    const auto & point = message->points.front();
    if (message->joint_names.size() != point.positions.size()) {
      arm_valid_ = false;
      publish_status("arm_rejected=name_position_count", true);
      return;
    }
    std::array<bool, 10> seen{};
    std::array<double, 10> candidate{};
    for (std::size_t index = 0; index < message->joint_names.size(); ++index) {
      const double value = point.positions[index];
      if (!finite(value)) {
        arm_valid_ = false;
        publish_status("arm_rejected=nonfinite", true);
        return;
      }
      std::size_t arm_index = 10;
      for (std::size_t candidate_index = 0; candidate_index < 10; ++candidate_index) {
        if (message->joint_names[index] == kArmSdkJointNames[candidate_index]) {
          arm_index = candidate_index;
          break;
        }
      }
      if (arm_index >= 10 || seen[arm_index]) {
        arm_valid_ = false;
        publish_status("arm_rejected=unknown_or_duplicate_joint", true);
        return;
      }
      candidate[arm_index] = value;
      seen[arm_index] = true;
    }
    if (!std::all_of(seen.begin(), seen.end(), [](bool value) {return value;})) {
      arm_valid_ = false;
      publish_status("arm_rejected=all_ten_arm_joints_required", true);
      return;
    }
    latest_arm_ = candidate;
    arm_valid_ = true;
    arm_arrival_ = SteadyClock::now();
    ++arm_sequence_;
  }

  void on_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    if (!message) {
      velocity_valid_ = false;
      return;
    }
    const auto & twist = message->twist;
    const std::array<double, 6> all = {
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z};
    if (!std::all_of(all.begin(), all.end(), finite) ||
      std::abs(twist.linear.z) > 1.0e-6 ||
      std::abs(twist.angular.x) > 1.0e-6 ||
      std::abs(twist.angular.y) > 1.0e-6)
    {
      velocity_valid_ = false;
      publish_status("velocity_rejected=nonfinite_or_unsupported_axis", true);
      return;
    }
    latest_velocity_ = {twist.linear.x, twist.linear.y, twist.angular.z};
    velocity_valid_ = true;
    velocity_arrival_ = SteadyClock::now();
    // This topic is the already-processed dry-run pipeline output.  During
    // post-prepare re-arm, only a fresh neutral sample observed while the
    // Deadman is released may advance the gate.
    const auto previous_state = prepare_rearm_gate_.state();
    (void)prepare_rearm_gate_.observe_velocity(latest_velocity_);
    if (prepare_rearm_gate_.state() != previous_state) {
      publish_status(
        "prepare_rearm=" + prepare_rearm_gate_.state_name(), true);
    }
  }

  void on_deadman(const std_msgs::msg::Bool::SharedPtr message)
  {
    const bool previous = deadman_active_;
    deadman_seen_ = static_cast<bool>(message);
    deadman_active_ = message && message->data;
    deadman_arrival_ = SteadyClock::now();
    const auto previous_rearm_state = prepare_rearm_gate_.state();
    // Exhibition is authorized by session_armed, not by the active/hold topic.
    // A normal LOCK publishes active=false while retaining that authority;
    // feeding it to the commissioning Deadman gate would erase readiness and
    // incorrectly escalate the next sequential-control tick to KILL.
    const bool became_ready = !exhibition_session_mode_ &&
      prepare_rearm_gate_.observe_deadman(deadman_active_);
    if (became_ready) {
      // Samples received before the new post-prepare press must never be
      // replayed.  Both enabled command streams have to publish again.
      clear_command_freshness();
    }
    if (prepare_rearm_gate_.state() != previous_rearm_state) {
      publish_status(
        "prepare_rearm=" + prepare_rearm_gate_.state_name(), true);
    }
    if (!exhibition_session_mode_ && previous && !deadman_active_ &&
      (outputs_active() || prepare_in_progress_ || head_recenter_requested()))
    {
      safe_stop_outputs(
        head_recenter_requested() ?
        "head_ownership_probe_failed cause=deadman_released" :
        "deadman_released", true);
    }
  }

  void on_session_armed(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!exhibition_session_mode_ || !message) {
      return;
    }
    const bool previous = session_control_armed_;
    session_control_seen_ = true;
    session_control_armed_ = message->data;
    if (previous == session_control_armed_) {
      return;
    }
    publish_status(
      "exhibition_session_armed=" + bool_text(session_control_armed_), true);
    if (!session_control_armed_ &&
      (outputs_active() || prepare_in_progress_ || prepared_))
    {
      safe_stop_outputs("exhibition_session_disarmed", true);
      return;
    }
    if (session_control_armed_) {
      // Commands captured before the explicit session-arm edge are not
      // reusable. Fresh VR/controller data must arrive before any output.
      clear_command_freshness();
    }
  }

  void on_kill(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message) {
      return;
    }
    const bool was_locally_latched = local_kill_latched_;
    kill_seen_ = true;
    kill_signal_active_ = message->data;
    kill_arrival_ = SteadyClock::now();
    if (kill_signal_active_) {
      local_kill_latched_ = true;
      // The normal fail-closed path asks the central supervisor to assert its
      // kill topic after it has already held/released every owned output.  Do
      // not immediately overwrite that primary failure with the supervisor's
      // acknowledgement, and do not repeat successful cleanup on every
      // latched kill sample.  A genuinely new external kill, or a pending
      // non-prepare cleanup debt, still enters the full stop path.
    const bool cleanup_retry_due = !prepare_in_progress_ && outputs_active();
      if (!was_locally_latched || cleanup_retry_due) {
        safe_stop_outputs(
          head_recenter_requested() ?
          "head_ownership_probe_failed cause=kill_signal_active" :
          "kill_signal_active", true, false);
      }
    }
  }

  void on_prepare(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    (void)request;
    if (prepare_in_progress_) {
      response->success = false;
      response->message = "prepare blocked: confirmation already in progress";
      return;
    }
    if (outputs_active()) {
      response->success = false;
      response->message = "prepare blocked: stop active outputs first";
      return;
    }
    AuthorizationInput input = authorization_input(enable_prepare_, true, false);
    const GateDecision gate = authorize_live_send(input);
    if (!gate.allowed) {
      response->success = false;
      response->message = "prepare blocked: " + gate.reason;
      publish_status(response->message, true);
      return;
    }
    const TransportResult initialized = ensure_transport();
    if (!initialized.ok) {
      response->success = false;
      response->message = initialized.detail;
      safe_stop_outputs("prepare_transport_failed:" + initialized.detail, true);
      return;
    }
    // ChannelFactory/LocoClient initialization may block long enough for the
    // kill, Deadman, or LowState samples that authorized entry to expire.  Do
    // not cross the StandUp boundary on the pre-initialization decision.
    const GateDecision post_initialize_gate = authorize_live_send(
      authorization_input(enable_prepare_, true, false));
    if (!post_initialize_gate.allowed) {
      response->success = false;
      response->message =
        "prepare blocked after transport initialization: " + post_initialize_gate.reason;
      safe_stop_outputs(
        "prepare_post_initialize_gate_failed:" + post_initialize_gate.reason, true);
      return;
    }
    prepared_ = false;
    prepare_rearm_gate_.reset();
    reset_head_recenter_state();
    clear_arm_feedback_follow();
    clear_command_freshness();
    exhibition_hold_active_ = false;
    exhibition_dropout_started_ = SteadyClock::time_point{};
    prepare_cancel_requested_.store(false);
    // Publish the ownership state before the worker can begin.  std::async may
    // start immediately; marking this afterwards creates a short window in
    // which watchdog/stop paths could believe that no SDK call owns LocoClient.
    prepare_in_progress_ = true;
    try {
      // StandUp/FSM polling may take several seconds.  Keep it off the
      // single-threaded ROS executor so kill, Deadman, and watchdog callbacks
      // continue to run.  No other transport call is allowed while this
      // worker owns the SDK client.
      prepare_future_ = std::async(std::launch::async, [this]() {
          return transport_->prepare([this]() {
              return prepare_cancel_requested_.load();
            }, prepare_enter_locomotion_, static_prepare_mode_, prepare_speed_mode_);
        });
    } catch (const std::exception & exception) {
      prepare_in_progress_ = false;
      prepare_cancel_requested_.store(true);
      response->success = false;
      response->message = std::string("prepare worker failed: ") + exception.what();
      safe_stop_outputs(response->message, true);
      return;
    }
    response->success = true;
    response->message =
      "prepare started asynchronously; wait for stable FSM confirmation status";
    publish_armed();
    publish_status(
      "prepare_in_progress=true waiting_for=stable_fsm_4", true);
  }

  void on_stop(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    (void)request;
    // A stop is latched locally.  Otherwise a still-streaming VR command
    // could resume motion on the very next timer tick without an explicit
    // reset/re-arm step.
    safe_stop_outputs("operator_stop", true, false);
    response->success = !outputs_active() && !prepare_in_progress_;
    response->message = response->success ?
      "writer outputs stopped, ArmSdk released, and local stop latched; "
      "no damping or torque-mode command sent" :
      "local stop latched, but SDK cleanup/cancel is still pending; use the physical E-stop";
  }

  void on_kill_service(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    (void)request;
    local_kill_latched_ = true;
    publish_kill_request();
    safe_stop_outputs("operator_kill_service", true, false);
    response->success = !outputs_active() && !prepare_in_progress_;
    response->message = response->success ?
      "local kill latched, global kill requested, and SDK cleanup completed" :
      "local/global kill requested, but SDK cleanup/cancel is still pending; "
      "press the physical E-stop";
  }

  void on_recenter_head(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    (void)request;
    response->success = false;
    response->message =
      "full physical recenter is suspended; use "
      "/r1/live_writer/probe_head_ownership for the bounded ownership micro-probe";
    publish_status("head_recenter=blocked physical_full_recenter_suspended=true", true);
  }

  void on_probe_head_ownership(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    (void)request;
    if (!head_recenter_enabled_) {
      response->success = false;
      response->message =
        "head ownership probe blocked: launch the separate probe-only mode";
      return;
    }
    if (!head_ownership_probe_only_) {
      response->success = false;
      response->message =
        "head ownership probe blocked: full physical recenter is suspended";
      return;
    }
    if (!head_ownership_probe_confirmed_ ||
      !head_ownership_probe_environment_confirmed())
    {
      response->success = false;
      response->message =
        "head ownership probe blocked: dedicated parameter/environment "
        "acknowledgement is missing";
      return;
    }
    if (head_recenter_state_ != HeadRecenterState::Idle || outputs_active() ||
      prepare_in_progress_)
    {
      response->success = false;
      response->message = "head ownership probe blocked: another output operation is active";
      return;
    }
    const auto now = SteadyClock::now();
    const GateDecision gate = authorize_live_send(
      authorization_input(true, enabled_commands_fresh(now), true));
    if (!gate.allowed) {
      response->success = false;
      response->message = "head ownership probe blocked: " + gate.reason;
      return;
    }
    const std::array<double, 2> measured{
      latest_state_[kHeadYawIndex], latest_state_[kHeadPitchIndex]};
    if (!head_within_absolute_limits(measured, head_absolute_limits_) ||
      !head_velocity_valid_ ||
      std::abs(latest_head_velocity_[0]) > max_head_seed_velocity_rad_s_ ||
      std::abs(latest_head_velocity_[1]) > max_head_seed_velocity_rad_s_)
    {
      response->success = false;
      response->message =
        "head ownership probe blocked: physical head seed is invalid, outside the official "
        "envelope, or moving";
      return;
    }
    head_recenter_state_ = HeadRecenterState::PendingFreshSeed;
    head_probe_terminal_status_.clear();
    head_recenter_request_state_sequence_ = state_sequence_;
    head_recenter_request_head_sequence_ = head_sequence_;
    head_recenter_last_state_sequence_ = state_sequence_;
    head_recenter_stable_seed_samples_ = 0;
    head_recenter_seed_candidate_yaw_pitch_ = {0.0, 0.0};
    head_recenter_confirmation_samples_ = 0;
    head_recenter_request_time_ = now;
    head_recenter_started_time_ = SteadyClock::time_point{};
    head_recenter_follow_wait_since_ = SteadyClock::time_point{};
    head_probe_phase_started_time_ = SteadyClock::time_point{};
    head_probe_weight_ = 0.0;
    head_probe_direction_ = 0.0;
    head_probe_step_magnitude_rad_ = 0.0;
    head_probe_feedback_samples_ = 0;
    head_probe_last_feedback_progress_rad_ = 0.0;
    head_probe_outbound_baseline_progress_rad_ = 0.0;
    response->success = true;
    response->message =
      "head ownership micro-probe accepted; no recenter will be performed; "
      "waiting for new stable feedback before zero-weight ArmSdk discovery";
    publish_status(
      "head_ownership_probe=pending_fresh_seed no_recenter=true current_yaw=" +
      std::to_string(measured[0]) + " current_pitch=" +
      std::to_string(measured[1]), true);
  }

  void on_reset_kill(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    (void)request;
    const auto now = SteadyClock::now();
    if (!kill_seen_ || kill_signal_active_ || age(now, kill_arrival_) > kill_timeout_sec_) {
      response->success = false;
      response->message = "reset blocked: external kill must be explicitly false and fresh";
      return;
    }
    if (outputs_active() || prepare_in_progress_) {
      response->success = false;
      response->message = "reset blocked: stop outputs/prepare first";
      return;
    }
    if (send_commands_) {
      const LiveEnvironment environment = LiveEnvironment::from_process();
      if (!live_environment_complete(environment) ||
        !commissioning_parameter_ok(environment) ||
        !control_source_ok(environment))
      {
        response->success = false;
        response->message = "reset blocked: live environment/commissioning gate incomplete";
        return;
      }
    }
    local_kill_latched_ = false;
    // Replace this writer's retained assert request with false. The central
    // supervisor ignores false requests, but a later supervisor restart must
    // not replay an already-reset true sample.
    publish_kill_request(false);
    response->success = true;
    response->message = "local kill latch reset; every send remains independently gated";
    publish_status("local_kill_latch=reset", true);
  }

  void on_timer()
  {
    const auto now = SteadyClock::now();
    exhibition_arm_hold_sent_this_tick_ = false;
    const double dt = last_tick_ == SteadyClock::time_point{} ?
      0.0 : std::clamp(age(now, last_tick_), 0.0, 0.10);
    last_tick_ = now;

    finish_prepare_if_ready();
    if (prepare_in_progress_) {
      const std::string failure = prepare_watchdog_failure(now);
      if (!failure.empty() && !prepare_cancel_requested_.load()) {
        safe_stop_outputs("prepare_" + failure, true);
      }
      publish_debug_head({0.0, 0.0});
      publish_debug_velocity({0.0, 0.0, 0.0});
      publish_armed();
      publish_periodic_status(now);
      return;
    }

    // Static prepare has no head/arm/velocity feature, and an exhibition
    // session has a short post-prepare interval before its first fresh output.
    // Both already crossed a physical StandUp/Start boundary, so retain the
    // KILL, LowState, motor-health and environment watchdog until STOP clears
    // prepared_, even while outputs_active() is still false.
    const bool safety_watchdog_required =
      outputs_active() ||
      (prepared_ && (static_prepare_mode_ || exhibition_session_mode_));
    if (safety_watchdog_required) {
      const std::string stale_reason = active_watchdog_failure(now);
      if (!stale_reason.empty()) {
        safe_stop_outputs(
          head_recenter_requested() ?
          "head_ownership_probe_failed cause=" + stale_reason : stale_reason,
          true);
        return;
      }
      const bool exhibition_dropout = exhibition_session_mode_ &&
        ((enable_head_ && !head_fresh(now)) ||
        (enable_arms_ && !arm_fresh(now)) ||
        (enable_locomotion_ && !velocity_fresh(now)));
      if (exhibition_dropout) {
        if (!exhibition_hold_active_) {
          exhibition_hold_active_ = true;
          exhibition_dropout_started_ = now;
          publish_status(
            "exhibition_vr=RECONNECTING action=arms_hold_locomotion_zero", true);
        }
      } else if (exhibition_hold_active_) {
        exhibition_hold_active_ = false;
        exhibition_dropout_started_ = SteadyClock::time_point{};
        publish_status("exhibition_vr=RECOVERED automatic_resume=true", true);
      }
      const std::string arm_follow_reason = arm_feedback_follow_failure(now);
      if (!arm_follow_reason.empty()) {
        safe_stop_outputs(arm_follow_reason, true);
        return;
      }
    }

    const bool sequential = sequential_control_enabled_ && send_commands_;
    const bool upper_allowed = !sequential || process_sequential_control(now);
    const bool upper_was_claimed = head_claimed_;
    if (enable_head_ && upper_allowed) {
      if (automatic_head_center_active_) {
        process_automatic_exhibition_head_center(now, dt);
      } else if (head_recenter_enabled_) {
        process_head_recenter(now, dt);
      } else {
        process_head(now, dt);
      }
    } else {
      publish_debug_head({0.0, 0.0});
    }
    // The automatic startup center owns the shared 13-field ArmSdk frame.
    // Keep arms at their captured physical seed and locomotion at zero until
    // fresh feedback confirms the head reached the centered tracking window.
    if (sequential && head_claimed_ && !upper_was_claimed) {
      // Head seeds the shared frame first. Rebase the arm rate limiter too,
      // otherwise its pre-walk q would drag a freshly measured seed backwards.
      std::copy_n(held_head_target_.begin(), 10, last_arm_command_.begin());
      arm_neutral_hold_.reset();
    }
    if (enable_arms_ && upper_allowed && !automatic_head_center_active_ &&
      (!sequential || head_claimed_))
    {
      process_arms(now, dt);
    } else {
      publish_debug_arm({});
    }
    if (enable_locomotion_ && !automatic_head_center_active_ &&
      (!sequential || sequential_control_.phase() == SequentialControl::Phase::Walk))
    {
      process_velocity(now, dt);
    } else {
      publish_debug_velocity({0.0, 0.0, 0.0});
    }
    publish_armed();
    publish_periodic_status(now);
  }

  bool process_sequential_control(const SteadyClock::time_point & now)
  {
    using Phase = SequentialControl::Phase;
    // Startup head centering retains its existing exclusive, bounded path.
    if (automatic_head_center_active_) {return true;}
    const auto gate = authorize_live_send(authorization_input(true, true, true));
    if (!gate.allowed) {
      if (outputs_active()) {safe_stop_outputs("sequential_gate:" + gate.reason, true);}
      return false;
    }
    const auto initialized = ensure_transport();
    if (!initialized.ok) {
      safe_stop_outputs("sequential_initialize:" + initialized.detail, true);
      return false;
    }
    LocoModeSample mode;
    try {mode = transport_->poll_loco_mode();}
    catch (const std::exception & error) {
      safe_stop_outputs("sequential_mode_reader:" + std::string(error.what()), true);
      return false;
    }
    const double seconds = std::chrono::duration<double>(now.time_since_epoch()).count();
    const auto previous = sequential_control_.phase();
    const std::string previous_name = sequential_control_.name();
    const double previous_started = sequential_control_.phase_started();
    sequential_control_.update({
      seconds, velocity_fresh(now) && !is_zero(latest_velocity_),
      enabled_commands_fresh(now), head_claimed_, mode.valid, mode.fsm,
      std::chrono::duration<double>(mode.requested_at.time_since_epoch()).count(),
      state_fresh(now) && legs_quiet_, state_sequence_});
    if (sequential_control_.phase() == Phase::Fault) {
      safe_stop_outputs("sequential_" + sequential_control_.error(), true);
      return false;
    }
    if (sequential_control_.phase() != previous) {
      publish_status(
        "sequential_phase=" + std::string(sequential_control_.name()) +
        " previous=" + previous_name +
        " previous_duration_sec=" + std::to_string(
          previous_started > 0 ? seconds - previous_started : 0) +
        " fsm=" + std::to_string(mode.fsm), true);
      last_velocity_command_ = {0.0, 0.0, 0.0};
      sequential_zero_sent_ = SteadyClock::time_point{};
      if (sequential_control_.phase() == Phase::Release) {
        // Hand authority back from the measured pose using the vendor weight
        // ramp. Firmware selects its walking posture; no invented joint zeros.
        sequential_release_pose_ = latest_state_;
        clear_arm_feedback_follow();
      }
      if (sequential_control_.phase() == Phase::Upper) {
        reset_head_seed_settle();
        arm_neutral_hold_.reset();
        std::copy_n(latest_state_.begin(), 10, last_arm_command_.begin());
        last_head_yaw_pitch_ = {0.0, 0.0};
      }
    }
    const auto phase = sequential_control_.phase();
    if (phase == Phase::Upper) {return true;}
    if (phase == Phase::Walk) {return false;}

    // Refresh neutral wireless axes throughout handover, mode observation and
    // settling. A single queued zero is not accepted as evidence of stopping.
    if (sequential_zero_sent_ == SteadyClock::time_point{} ||
      age(now, sequential_zero_sent_) >= 0.05)
    {
      const auto zero = transport_->set_velocity(0.0, 0.0, 0.0, command_duration_sec_);
      if (!zero.ok) {
        safe_stop_outputs("sequential_zero:" + zero.detail, true);
        return false;
      }
      sequential_zero_sent_ = now;
      sequential_control_.zero_sent();
      locomotion_active_ = false;
    }
    if (phase == Phase::Release) {
      if (sequential_control_.release_complete(seconds)) {
        const auto released = transport_->finish_head_handover();
        if (!released.ok) {
          safe_stop_outputs("sequential_release_finish:" + released.detail, true);
          return false;
        }
        head_claimed_ = false;
        arm_neutral_hold_.reset();
        sequential_control_.release_finished(seconds);
        publish_status("sequential_phase=waiting_811 upper_ownership=released", true);
      } else if (sequential_control_.release_due(seconds)) {
        const auto released = transport_->command_head_weighted(
          sequential_release_pose_, sequential_control_.release_weight());
        if (!released.ok) {
          safe_stop_outputs("sequential_release:" + released.detail, true);
          return false;
        }
        sequential_control_.release_frame_sent(seconds);
      }
    }
    return false;
  }

  void maintain_exhibition_arm_hold(const std::string & source)
  {
    if (!exhibition_session_mode_ || !send_commands_ || !head_claimed_ ||
      !transport_ || !transport_->initialized() || exhibition_arm_hold_sent_this_tick_)
    {
      return;
    }
    exhibition_arm_hold_sent_this_tick_ = true;
    // Keep any pending feedback-follow obligation for the last meaningful
    // arm target.  Re-sending that exact safe hold must not hide a controller
    // ownership failure which began immediately before tracking disappeared.
    const TransportResult held = transport_->hold_head(held_head_target_);
    if (!held.ok) {
      safe_stop_outputs(
        "exhibition_arm_hold_failed source=" + source + ":" + held.detail, true);
    }
  }

  void maintain_exhibition_velocity_zero(const std::string & source)
  {
    publish_debug_velocity({0.0, 0.0, 0.0});
    last_velocity_command_ = {0.0, 0.0, 0.0};
    if (!exhibition_session_mode_ || !send_commands_ || !locomotion_active_ ||
      !transport_ || !transport_->initialized())
    {
      return;
    }
    // SetVelocity carries a bounded duration, so a single explicit zero is a
    // recoverable stop that keeps the official locomotion controller ready.
    // Operator STOP/KILL still uses the stronger StopMove cleanup path.
    const TransportResult zeroed = transport_->set_velocity(
      0.0, 0.0, 0.0, command_duration_sec_);
    if (!zeroed.ok) {
      safe_stop_outputs(
        "exhibition_velocity_zero_failed source=" + source + ":" +
        zeroed.detail, true);
      return;
    }
    locomotion_active_ = false;
  }

  bool head_recenter_requested() const
  {
    return head_recenter_state_ != HeadRecenterState::Disabled &&
           head_recenter_state_ != HeadRecenterState::Idle;
  }

  void reset_head_recenter_state()
  {
    head_recenter_state_ = head_recenter_enabled_ ?
      HeadRecenterState::Idle : HeadRecenterState::Disabled;
    automatic_head_center_active_ = false;
    automatic_head_center_complete_ = false;
    automatic_head_center_paused_ = false;
    head_recenter_stable_seed_samples_ = 0;
    head_recenter_confirmation_samples_ = 0;
    head_recenter_request_time_ = SteadyClock::time_point{};
    head_recenter_started_time_ = SteadyClock::time_point{};
    head_recenter_last_advance_time_ = SteadyClock::time_point{};
    head_recenter_follow_wait_since_ = SteadyClock::time_point{};
    head_probe_phase_started_time_ = SteadyClock::time_point{};
    head_probe_weight_ = 0.0;
    head_probe_direction_ = 0.0;
    head_probe_step_magnitude_rad_ = 0.0;
    head_probe_feedback_samples_ = 0;
    head_probe_last_feedback_progress_rad_ = 0.0;
    head_probe_outbound_baseline_progress_rad_ = 0.0;
    head_probe_full_weight_seed_valid_ = false;
  }

  const ArmSdkPositions & head_probe_other_joint_reference() const
  {
    const bool handoff_in_progress =
      head_recenter_state_ == HeadRecenterState::ZeroWeightDiscovery ||
      head_recenter_state_ == HeadRecenterState::RampingWeight;
    return !handoff_in_progress && head_probe_full_weight_seed_valid_ ?
      head_probe_full_weight_seed_ : head_recenter_seed_;
  }

  double head_probe_other_joint_limit(std::size_t index) const
  {
    // Running/FSM 811 continuously makes small upper-body balance
    // corrections.  While ownership is blended in, follow those measured
    // corrections and allow only a bounded 0.02-rad-class handoff envelope
    // for the arms and waist.  Once full ArmSdk ownership is reached, capture
    // the settled pose and restore the original tight 0.003-rad guard for the
    // actual yaw probe.  Head pitch never receives the relaxed envelope.
    if (head_recenter_state_ == HeadRecenterState::RampingWeight &&
      index <= kWaistYawIndex)
    {
      return head_probe_ramp_other_joint_tolerance_rad_;
    }
    return head_probe_other_joint_tolerance_rad_;
  }

  std::string head_probe_diagnostics(
    const std::string & event, const ArmSdkPositions & measured) const
  {
    std::size_t max_other_index = 0;
    double max_other_delta = 0.0;
    double max_other_limit = head_probe_other_joint_tolerance_rad_;
    const ArmSdkPositions & other_reference = head_probe_other_joint_reference();
    for (std::size_t index = 0; index < measured.size(); ++index) {
      if (index == kHeadYawIndex) {
        continue;
      }
      const double delta = std::abs(measured[index] - other_reference[index]);
      if (delta > max_other_delta) {
        max_other_delta = delta;
        max_other_index = index;
        max_other_limit = head_probe_other_joint_limit(index);
      }
    }
    const double progress = head_probe_direction_ *
      (measured[kHeadYawIndex] - head_recenter_seed_[kHeadYawIndex]);
    const double outbound_progress =
      progress - head_probe_outbound_baseline_progress_rad_;
    const double actual_excursion =
      std::abs(measured[kHeadYawIndex] - head_recenter_seed_[kHeadYawIndex]);
    const double actual_excursion_limit =
      head_probe_step_magnitude_rad_ + head_probe_direction_tolerance_rad_;
    std::ostringstream detail;
    detail << "head_ownership_probe=" << event
           << " phase=" << head_recenter_state_name(head_recenter_state_)
           << " weight=" << head_probe_weight_
           << " target_yaw=" << head_recenter_target_yaw_pitch_[0]
           << " target_pitch=" << head_recenter_target_yaw_pitch_[1]
           << " feedback_yaw=" << measured[kHeadYawIndex]
           << " feedback_pitch=" << measured[kHeadPitchIndex]
           << " error_yaw="
           << (measured[kHeadYawIndex] - head_recenter_target_yaw_pitch_[0])
           << " error_pitch="
           << (measured[kHeadPitchIndex] - head_recenter_target_yaw_pitch_[1])
           << " progress_yaw=" << progress
           << " outbound_baseline_yaw="
           << head_probe_outbound_baseline_progress_rad_
           << " outbound_progress_yaw=" << outbound_progress
           << " actual_excursion_yaw=" << actual_excursion
           << " actual_excursion_limit=" << actual_excursion_limit
           << " feedback_yaw_velocity=" << latest_head_velocity_[0]
           << " feedback_pitch_velocity=" << latest_head_velocity_[1]
           << " feedback_velocity_limit="
           << head_probe_max_feedback_velocity_rad_s_
           << " max_other_joint=" << kArmSdkJointNames[max_other_index]
           << " max_other_delta=" << max_other_delta
           << " max_other_limit=" << max_other_limit
           << " full_weight_seed=" << bool_text(head_probe_full_weight_seed_valid_)
           << " feedback_samples=" << head_probe_feedback_samples_;
    return detail.str();
  }

  std::string head_probe_feedback_failure(const ArmSdkPositions & measured) const
  {
    if (!sane_arm_sdk_feedback(measured)) {
      return "nonfinite_or_outside_arm_sdk_sanity_envelope";
    }
    if (!head_velocity_valid_ || !finite_values(latest_head_velocity_)) {
      return "head_velocity_missing_or_nonfinite";
    }
    if (std::abs(latest_head_velocity_[0]) >
      head_probe_max_feedback_velocity_rad_s_ ||
      std::abs(latest_head_velocity_[1]) >
      head_probe_max_feedback_velocity_rad_s_)
    {
      return "feedback_velocity_exceeded";
    }
    const ArmSdkPositions & other_reference = head_probe_other_joint_reference();
    for (std::size_t index = 0; index < measured.size(); ++index) {
      if (index == kHeadYawIndex) {
        continue;
      }
      const double delta = std::abs(measured[index] - other_reference[index]);
      const double limit = head_probe_other_joint_limit(index);
      if (delta > limit) {
        std::ostringstream detail;
        detail << "unexpected_other_joint_motion joint=" << kArmSdkJointNames[index]
               << " delta=" << delta
               << " limit=" << limit
               << " phase=" << head_recenter_state_name(head_recenter_state_);
        return detail.str();
      }
    }
    return {};
  }

  void fail_head_ownership_probe(
    const std::string & cause, const ArmSdkPositions & measured)
  {
    safe_stop_outputs(
      "head_ownership_probe_failed cause=" + cause + " " +
      head_probe_diagnostics("failed", measured), true);
  }

  bool publish_head_probe_frame(double weight)
  {
    const auto command_now = SteadyClock::now();
    const GateDecision gate = authorize_live_send(
      authorization_input(true, enabled_commands_fresh(command_now), true));
    if (!gate.allowed || !state_fresh(command_now) || !head_fresh(command_now)) {
      fail_head_ownership_probe(
        "command_gate_failed:" +
        (gate.allowed ? std::string("freshness_expired") : gate.reason),
        latest_state_);
      return false;
    }
    if (!finite(weight) || weight < 0.0 || weight > 1.0 ||
      !finite_values(head_recenter_target_yaw_pitch_) ||
      !head_within_absolute_limits(
        head_recenter_target_yaw_pitch_, head_absolute_limits_))
    {
      fail_head_ownership_probe("invalid_weight_or_target", latest_state_);
      return false;
    }
    // ArmSdk's single weight covers all 13 upper-body joints.  During the
    // ownership ramp, freezing the ten arm joints and waist at the beginning
    // of the probe makes the SDK fight the running balance controller over
    // harmless encoder settling.  Pass every non-head joint through from the
    // newest measured frame, then overwrite only the two explicitly probed
    // head targets.  The existing 0.003 rad excursion guard still fails closed
    // if any non-yaw joint actually moves away from the stable seed.
    const bool handoff_in_progress =
      head_recenter_state_ == HeadRecenterState::ZeroWeightDiscovery ||
      head_recenter_state_ == HeadRecenterState::RampingWeight;
    if (handoff_in_progress) {
      held_head_target_ = latest_state_;
    } else if (head_probe_full_weight_seed_valid_) {
      held_head_target_ = head_probe_full_weight_seed_;
    } else {
      fail_head_ownership_probe("missing_full_weight_seed", latest_state_);
      return false;
    }
    held_head_target_[kHeadPitchIndex] = head_recenter_target_yaw_pitch_[1];
    held_head_target_[kHeadYawIndex] = head_recenter_target_yaw_pitch_[0];
    head_probe_weight_ = weight;
    const TransportResult result = transport_->command_head_weighted(
      held_head_target_, weight);
    if (!result.ok) {
      fail_head_ownership_probe("weighted_send_failed:" + result.detail, latest_state_);
      return false;
    }
    publish_debug_head(head_recenter_target_yaw_pitch_);
    return true;
  }

  void process_head_recenter(const SteadyClock::time_point & now, double dt)
  {
    (void)dt;
    if (!head_recenter_requested()) {
      publish_debug_head({0.0, 0.0});
      return;
    }
    if (!head_ownership_probe_only_) {
      fail_head_ownership_probe("probe_only_interlock_lost", latest_state_);
      return;
    }
    if (head_recenter_state_ == HeadRecenterState::PendingFreshSeed &&
      age(now, head_recenter_request_time_) > 2.0)
    {
      fail_head_ownership_probe("fresh_seed_timeout", latest_state_);
      return;
    }
    if (head_recenter_started_time_ != SteadyClock::time_point{} &&
      age(now, head_recenter_started_time_) > head_probe_total_timeout_sec_)
    {
      fail_head_ownership_probe("total_timeout", latest_state_);
      return;
    }
    if (!head_fresh(now) || !state_fresh(now) || !control_permission_fresh(now) ||
      !kill_clear_fresh(now) || !head_ownership_probe_confirmed_ ||
      !head_ownership_probe_environment_confirmed())
    {
      fail_head_ownership_probe("input_stale_or_interlock_lost", latest_state_);
      return;
    }
    const GateDecision gate = authorize_live_send(
      authorization_input(true, enabled_commands_fresh(now), true));
    if (!gate.allowed) {
      fail_head_ownership_probe("gate_failed:" + gate.reason, latest_state_);
      return;
    }

    const std::array<double, 2> measured_head{
      latest_state_[kHeadYawIndex], latest_state_[kHeadPitchIndex]};
    if (!head_within_absolute_limits(measured_head, head_absolute_limits_)) {
      fail_head_ownership_probe("feedback_outside_absolute_envelope", latest_state_);
      return;
    }

    if (head_recenter_state_ == HeadRecenterState::PendingFreshSeed) {
      if (state_sequence_ <= head_recenter_request_state_sequence_ ||
        head_sequence_ <= head_recenter_request_head_sequence_ ||
        state_sequence_ == head_recenter_last_state_sequence_)
      {
        publish_debug_head({0.0, 0.0});
        return;
      }
      head_recenter_last_state_sequence_ = state_sequence_;
      if (!head_velocity_valid_ || !finite_values(latest_head_velocity_) ||
        std::abs(latest_head_velocity_[0]) > max_head_seed_velocity_rad_s_ ||
        std::abs(latest_head_velocity_[1]) > max_head_seed_velocity_rad_s_)
      {
        head_recenter_stable_seed_samples_ = 0;
        publish_debug_head({0.0, 0.0});
        return;
      }
      if (head_recenter_stable_seed_samples_ == 0) {
        head_probe_seed_candidate_ = latest_state_;
        head_recenter_stable_seed_samples_ = 1;
      } else {
        bool stable = true;
        for (std::size_t index = 0; index < latest_state_.size(); ++index) {
          if (std::abs(latest_state_[index] - head_probe_seed_candidate_[index]) >
            head_probe_seed_stability_rad_)
          {
            stable = false;
            break;
          }
        }
        if (!stable) {
          head_probe_seed_candidate_ = latest_state_;
          head_recenter_stable_seed_samples_ = 1;
          publish_debug_head({0.0, 0.0});
          return;
        }
        ++head_recenter_stable_seed_samples_;
      }
      if (head_recenter_stable_seed_samples_ <
        static_cast<std::size_t>(head_probe_seed_samples_))
      {
        publish_debug_head({0.0, 0.0});
        return;
      }

      const TransportResult initialized = ensure_transport();
      if (!initialized.ok) {
        fail_head_ownership_probe(
          "transport_initialize_failed:" + initialized.detail, latest_state_);
        return;
      }
      const auto send_now = SteadyClock::now();
      const GateDecision send_gate = authorize_live_send(
        authorization_input(true, enabled_commands_fresh(send_now), true));
      if (!send_gate.allowed || !state_fresh(send_now) || !head_fresh(send_now) ||
        !head_ownership_probe_environment_confirmed())
      {
        fail_head_ownership_probe(
          "post_initialize_gate_failed:" +
          (send_gate.allowed ? std::string("freshness_or_probe_ack") : send_gate.reason),
          latest_state_);
        return;
      }
      const std::array<double, 2> seed{
        latest_state_[kHeadYawIndex], latest_state_[kHeadPitchIndex]};
      if (!head_within_absolute_limits(seed, head_absolute_limits_) ||
        !head_velocity_valid_ || !finite_values(latest_head_velocity_) ||
        std::abs(latest_head_velocity_[0]) > max_head_seed_velocity_rad_s_ ||
        std::abs(latest_head_velocity_[1]) > max_head_seed_velocity_rad_s_)
      {
        fail_head_ownership_probe("seed_changed_or_unstable", latest_state_);
        return;
      }
      for (std::size_t index = 0; index < latest_state_.size(); ++index) {
        if (std::abs(latest_state_[index] - head_probe_seed_candidate_[index]) >
          head_probe_seed_stability_rad_)
        {
          fail_head_ownership_probe(
            "full_seed_changed_before_zero_weight_boundary", latest_state_);
          return;
        }
      }
      // Prefer a step toward numerical zero when the seed has a meaningful
      // sign. FSM 811 on the commissioned R1 can, however, report yaw≈0 while
      // the head is visibly turned right. In that case choose the side with
      // more room inside the immutable manufacturer envelope. This keeps the
      // probe tiny and verifiable without treating encoder zero as a visual
      // centre.
      const double positive_room = head_absolute_limits_.yaw_max - seed[0];
      const double negative_room = seed[0] - head_absolute_limits_.yaw_min;
      const double signed_seed_threshold =
        head_probe_min_progress_rad_ + head_probe_direction_tolerance_rad_;
      if (seed[0] > signed_seed_threshold) {
        head_probe_direction_ = -1.0;
      } else if (seed[0] < -signed_seed_threshold) {
        head_probe_direction_ = 1.0;
      } else {
        head_probe_direction_ = positive_room >= negative_room ? 1.0 : -1.0;
      }
      const double available_room = head_probe_direction_ > 0.0 ?
        positive_room : negative_room;
      head_probe_step_magnitude_rad_ = std::min(
        head_probe_step_rad_, available_room);
      if (head_probe_step_magnitude_rad_ <
        head_probe_min_progress_rad_ + head_probe_direction_tolerance_rad_)
      {
        fail_head_ownership_probe("insufficient_yaw_room_for_probe", latest_state_);
        return;
      }
      head_recenter_seed_ = latest_state_;
      held_head_target_ = head_recenter_seed_;
      head_recenter_target_yaw_pitch_ = seed;
      head_origin_yaw_pitch_ = seed;
      head_probe_weight_ = 0.0;
      // Mark ownership before crossing even the zero-weight seed boundary: an
      // SDK error is not proof that the frame was not observed.
      head_claimed_ = true;
      const TransportResult seeded = transport_->seed_head_weighted(
        head_recenter_seed_, 0.0);
      if (!seeded.ok) {
        fail_head_ownership_probe("zero_weight_seed_failed:" + seeded.detail, latest_state_);
        return;
      }
      head_recenter_state_ = HeadRecenterState::ZeroWeightDiscovery;
      head_recenter_started_time_ = send_now;
      head_probe_phase_started_time_ = send_now;
      head_recenter_last_state_sequence_ = state_sequence_;
      head_probe_feedback_samples_ = 0;
      head_probe_last_feedback_progress_rad_ = 0.0;
      publish_debug_head(head_recenter_target_yaw_pitch_);
      publish_status(head_probe_diagnostics("zero_weight_discovery", latest_state_), true);
      return;
    }

    const bool new_state = state_sequence_ != head_recenter_last_state_sequence_;
    if (new_state) {
      head_recenter_last_state_sequence_ = state_sequence_;
      const std::string feedback_failure = head_probe_feedback_failure(latest_state_);
      if (!feedback_failure.empty()) {
        fail_head_ownership_probe(feedback_failure, latest_state_);
        return;
      }
      const double yaw_delta =
        latest_state_[kHeadYawIndex] - head_recenter_seed_[kHeadYawIndex];
      if ((head_recenter_state_ == HeadRecenterState::ZeroWeightDiscovery ||
        head_recenter_state_ == HeadRecenterState::RampingWeight) &&
        std::abs(yaw_delta) > head_probe_claim_tolerance_rad_)
      {
        fail_head_ownership_probe("seed_hold_yaw_moved", latest_state_);
        return;
      }
      if (head_recenter_state_ == HeadRecenterState::ProbingOut ||
        head_recenter_state_ == HeadRecenterState::VerifyingOut ||
        head_recenter_state_ == HeadRecenterState::ReturningToSeed ||
        head_recenter_state_ == HeadRecenterState::ConfirmingReturn)
      {
        const double progress = head_probe_direction_ * yaw_delta;
        if (progress < -head_probe_direction_tolerance_rad_) {
          fail_head_ownership_probe("yaw_moved_in_wrong_direction", latest_state_);
          return;
        }
        if (progress >
          head_probe_step_magnitude_rad_ + head_probe_direction_tolerance_rad_)
        {
          fail_head_ownership_probe("yaw_exceeded_probe_excursion", latest_state_);
          return;
        }
      }
    }

    if (head_recenter_state_ == HeadRecenterState::ZeroWeightDiscovery) {
      head_recenter_target_yaw_pitch_ = {
        head_recenter_seed_[kHeadYawIndex], head_recenter_seed_[kHeadPitchIndex]};
      if (!publish_head_probe_frame(0.0)) {
        return;
      }
      if (age(now, head_probe_phase_started_time_) >=
        head_probe_zero_weight_hold_sec_)
      {
        head_recenter_state_ = HeadRecenterState::RampingWeight;
        head_probe_phase_started_time_ = now;
        publish_status(head_probe_diagnostics("weight_ramp_started", latest_state_), true);
      }
      return;
    }

    if (head_recenter_state_ == HeadRecenterState::RampingWeight) {
      const double weight = std::clamp(
        age(now, head_probe_phase_started_time_) / head_probe_weight_ramp_sec_,
        0.0, 1.0);
      head_recenter_target_yaw_pitch_ = {
        head_recenter_seed_[kHeadYawIndex], head_recenter_seed_[kHeadPitchIndex]};
      if (!publish_head_probe_frame(weight)) {
        return;
      }
      if (weight >= 1.0) {
        // The Running controller may settle the arms/waist by a few
        // milliradians while its ownership is blended out.  Capture that
        // measured end-of-handoff pose and hold it from this point onward;
        // the yaw micro-probe then runs with the original tight non-yaw guard
        // relative to this settled full-weight pose.
        head_probe_full_weight_seed_ = latest_state_;
        head_probe_full_weight_seed_valid_ = true;
        held_head_target_ = head_probe_full_weight_seed_;
        held_head_target_[kHeadPitchIndex] = head_recenter_target_yaw_pitch_[1];
        held_head_target_[kHeadYawIndex] = head_recenter_target_yaw_pitch_[0];
        head_probe_outbound_baseline_progress_rad_ = head_probe_direction_ *
          (latest_state_[kHeadYawIndex] - head_recenter_seed_[kHeadYawIndex]);
        head_recenter_state_ = HeadRecenterState::ProbingOut;
        head_probe_phase_started_time_ = now;
        publish_status(head_probe_diagnostics("full_weight_seed_hold", latest_state_), true);
      }
      return;
    }

    if (head_recenter_state_ == HeadRecenterState::ProbingOut) {
      const double commanded_progress = std::min(
        head_probe_step_magnitude_rad_,
        head_probe_rate_rad_s_ * age(now, head_probe_phase_started_time_));
      head_recenter_target_yaw_pitch_ = {
        head_recenter_seed_[kHeadYawIndex] +
        head_probe_direction_ * commanded_progress,
        head_recenter_seed_[kHeadPitchIndex]};
      if (!publish_head_probe_frame(1.0)) {
        return;
      }
      if (commanded_progress >= head_probe_step_magnitude_rad_) {
        head_recenter_state_ = HeadRecenterState::VerifyingOut;
        head_probe_phase_started_time_ = now;
        head_probe_feedback_samples_ = 0;
        head_probe_last_feedback_progress_rad_ = 0.0;
        publish_status(head_probe_diagnostics("verifying_outbound_feedback", latest_state_), true);
      }
      return;
    }

    if (head_recenter_state_ == HeadRecenterState::VerifyingOut) {
      head_recenter_target_yaw_pitch_ = {
        head_recenter_seed_[kHeadYawIndex] +
        head_probe_direction_ * head_probe_step_magnitude_rad_,
        head_recenter_seed_[kHeadPitchIndex]};
      if (!publish_head_probe_frame(1.0)) {
        return;
      }
      if (new_state) {
        const double progress = head_probe_direction_ *
          (latest_state_[kHeadYawIndex] - head_recenter_seed_[kHeadYawIndex]);
        const double outbound_progress =
          progress - head_probe_outbound_baseline_progress_rad_;
        if (outbound_progress + head_probe_direction_tolerance_rad_ <
          head_probe_last_feedback_progress_rad_)
        {
          fail_head_ownership_probe("outbound_feedback_reversed", latest_state_);
          return;
        }
        if (outbound_progress >= head_probe_min_progress_rad_) {
          ++head_probe_feedback_samples_;
          head_probe_last_feedback_progress_rad_ = outbound_progress;
        } else {
          head_probe_feedback_samples_ = 0;
          head_probe_last_feedback_progress_rad_ =
            std::max(0.0, outbound_progress);
        }
        if (head_probe_feedback_samples_ >=
          static_cast<std::size_t>(head_probe_required_feedback_samples_))
        {
          head_recenter_state_ = HeadRecenterState::ReturningToSeed;
          head_probe_phase_started_time_ = now;
          head_probe_feedback_samples_ = 0;
          publish_status(
            head_probe_diagnostics("outbound_feedback_confirmed", latest_state_), true);
          return;
        }
      }
      if (age(now, head_probe_phase_started_time_) > head_probe_follow_timeout_sec_) {
        fail_head_ownership_probe("outbound_feedback_timeout", latest_state_);
      }
      return;
    }

    if (head_recenter_state_ == HeadRecenterState::ReturningToSeed) {
      const double remaining = std::max(
        0.0, head_probe_step_magnitude_rad_ -
        head_probe_rate_rad_s_ * age(now, head_probe_phase_started_time_));
      head_recenter_target_yaw_pitch_ = {
        head_recenter_seed_[kHeadYawIndex] + head_probe_direction_ * remaining,
        head_recenter_seed_[kHeadPitchIndex]};
      if (!publish_head_probe_frame(1.0)) {
        return;
      }
      if (remaining <= 0.0) {
        head_recenter_state_ = HeadRecenterState::ConfirmingReturn;
        head_probe_phase_started_time_ = now;
        head_probe_feedback_samples_ = 0;
        publish_status(head_probe_diagnostics("confirming_seed_return", latest_state_), true);
      }
      return;
    }

    if (head_recenter_state_ == HeadRecenterState::ConfirmingReturn) {
      head_recenter_target_yaw_pitch_ = {
        head_recenter_seed_[kHeadYawIndex], head_recenter_seed_[kHeadPitchIndex]};
      if (!publish_head_probe_frame(1.0)) {
        return;
      }
      if (new_state) {
        const bool returned =
          std::abs(
          latest_state_[kHeadYawIndex] - head_recenter_seed_[kHeadYawIndex]) <=
          head_probe_return_tolerance_rad_;
        head_probe_feedback_samples_ = returned ?
          head_probe_feedback_samples_ + 1 : 0;
        if (head_probe_feedback_samples_ >=
          static_cast<std::size_t>(head_probe_required_feedback_samples_))
        {
          const std::string completion_diagnostics =
            head_probe_diagnostics("verified", latest_state_);
          safe_stop_outputs(
            "head_ownership_probe_verified awaiting_release=true no_recenter=true " +
            completion_diagnostics, true);
          if (!outputs_active() && !prepare_in_progress_ && local_kill_latched_) {
            head_probe_terminal_status_ =
              "head_ownership_probe=passed terminal=true no_recenter=true "
              "release_complete=true kill_latched=true " + completion_diagnostics +
              " cleanup={" + last_safe_stop_detail_ + "}";
          } else {
            head_probe_terminal_status_ =
              "fail_closed reason=head_ownership_probe_failed "
              "cause=release_cleanup_pending terminal=true " + completion_diagnostics +
              " cleanup={" + last_safe_stop_detail_ + "}";
          }
          publish_status(head_probe_terminal_status_, true);
          return;
        }
      }
      if (age(now, head_probe_phase_started_time_) > head_probe_follow_timeout_sec_) {
        fail_head_ownership_probe("return_feedback_timeout", latest_state_);
      }
      return;
    }

    fail_head_ownership_probe("invalid_or_legacy_state_reached", latest_state_);
  }

  void begin_automatic_exhibition_head_center(
    const SteadyClock::time_point & now,
    const std::array<double, 2> & measured)
  {
    automatic_head_center_active_ = true;
    automatic_head_center_complete_ = false;
    automatic_head_center_paused_ = false;
    head_recenter_state_ = HeadRecenterState::PendingFreshSeed;
    head_recenter_request_state_sequence_ = state_sequence_;
    head_recenter_request_head_sequence_ = head_sequence_;
    head_recenter_last_state_sequence_ = state_sequence_;
    head_recenter_stable_seed_samples_ = 0;
    head_recenter_seed_candidate_yaw_pitch_ = {0.0, 0.0};
    head_recenter_confirmation_samples_ = 0;
    head_recenter_request_time_ = now;
    head_recenter_started_time_ = SteadyClock::time_point{};
    head_recenter_follow_wait_since_ = SteadyClock::time_point{};
    std::ostringstream detail;
    detail << "head_auto_center=pending"
           << " seed_yaw=" << measured[0]
           << " seed_pitch=" << measured[1]
           << " target_yaw=0 target_pitch=0";
    publish_status(detail.str(), true);
  }

  // A bounded automatic path used only while starting a full exhibition
  // session.  The public recenter service remains unavailable.  The path
  // waits for stationary feedback, preserves all non-head joints, clamps the
  // first target inward when LowState reports small stop overshoot, slews both
  // head joints toward zero, verifies feedback on every step, and keeps arms
  // and locomotion unavailable until five centered samples are observed.
  void process_automatic_exhibition_head_center(
    const SteadyClock::time_point & now, double dt)
  {
    (void)dt;
    if (!automatic_head_center_active_ || !head_recenter_requested()) {
      publish_debug_head({0.0, 0.0});
      return;
    }
    // LowState, motor health, the explicit RUN/LOCK session gate and KILL are
    // still hard interlocks.  VR command freshness is different in the
    // exhibition profile: a headset/controller radio dropout must hold the
    // exact last full ArmSdk frame and wait for reconnect instead of turning
    // a recoverable input interruption into a latched robot KILL.
    if (!state_fresh(now) || !motor_health_fresh(now) ||
      !control_permission_fresh(now) || !kill_clear_fresh(now))
    {
      safe_stop_outputs("head_recenter_input_missing_stale_or_interlocked", true);
      return;
    }
    if (!enabled_commands_fresh(now)) {
      if (!automatic_head_center_paused_) {
        automatic_head_center_paused_ = true;
        publish_status(
          "head_auto_center=paused reason=vr_reconnecting "
          "action=head_hold_arms_hold_locomotion_zero", true);
      }
      if (head_claimed_) {
        maintain_exhibition_arm_hold("head_auto_center_vr_reconnecting");
      }
      maintain_exhibition_velocity_zero("head_auto_center_vr_reconnecting");
      publish_debug_head(
        head_claimed_ ? head_recenter_target_yaw_pitch_ :
        std::array<double, 2>{0.0, 0.0});

      // Dropout time is not motion time.  Restart only the relevant bounded
      // windows so reconnect cannot create one large slew step or an
      // immediate following/overall timeout.  Physical feedback continues to
      // be checked by the hard watchdog above on every timer tick.
      if (head_recenter_state_ == HeadRecenterState::PendingFreshSeed) {
        head_recenter_request_time_ = now;
        head_recenter_request_state_sequence_ = state_sequence_;
        head_recenter_request_head_sequence_ = head_sequence_;
        head_recenter_last_state_sequence_ = state_sequence_;
        head_recenter_stable_seed_samples_ = 0;
      } else {
        head_recenter_started_time_ = now;
        head_recenter_last_advance_time_ = now;
        head_recenter_last_state_sequence_ = state_sequence_;
        head_recenter_follow_wait_since_ = SteadyClock::time_point{};
      }
      return;
    }
    if (automatic_head_center_paused_) {
      automatic_head_center_paused_ = false;
      head_recenter_last_advance_time_ = now;
      head_recenter_last_state_sequence_ = state_sequence_;
      head_recenter_follow_wait_since_ = SteadyClock::time_point{};
      publish_status("head_auto_center=resumed reason=vr_recovered", true);
    }
    if (head_recenter_state_ == HeadRecenterState::PendingFreshSeed &&
      age(now, head_recenter_request_time_) > 2.0)
    {
      safe_stop_outputs("head_recenter_fresh_seed_timeout", true);
      return;
    }
    if (head_recenter_started_time_ != SteadyClock::time_point{} &&
      age(now, head_recenter_started_time_) > head_recenter_timeout_sec_)
    {
      safe_stop_outputs("head_recenter_motion_timeout", true);
      return;
    }
    const GateDecision gate = authorize_live_send(
      authorization_input(true, enabled_commands_fresh(now), true));
    if (!gate.allowed) {
      safe_stop_outputs("head_recenter_gate_failed:" + gate.reason, true);
      return;
    }

    const std::array<double, 2> measured{
      latest_state_[kHeadYawIndex], latest_state_[kHeadPitchIndex]};
    if (!head_within_feedback_envelope(measured, head_absolute_limits_)) {
      safe_stop_outputs("head_recenter_feedback_outside_absolute_envelope", true);
      return;
    }

    if (head_recenter_state_ == HeadRecenterState::PendingFreshSeed) {
      if (state_sequence_ <= head_recenter_request_state_sequence_ ||
        head_sequence_ <= head_recenter_request_head_sequence_ ||
        state_sequence_ == head_recenter_last_state_sequence_)
      {
        publish_debug_head({0.0, 0.0});
        return;
      }
      head_recenter_last_state_sequence_ = state_sequence_;
      if (!head_velocity_valid_ ||
        std::abs(latest_head_velocity_[0]) > max_head_seed_velocity_rad_s_ ||
        std::abs(latest_head_velocity_[1]) > max_head_seed_velocity_rad_s_)
      {
        head_recenter_stable_seed_samples_ = 0;
        publish_debug_head({0.0, 0.0});
        return;
      }
      if (head_recenter_stable_seed_samples_ == 0) {
        head_recenter_seed_candidate_yaw_pitch_ = measured;
        head_recenter_stable_seed_samples_ = 1;
      } else if (
        std::abs(measured[0] - head_recenter_seed_candidate_yaw_pitch_[0]) >
        head_recenter_seed_stability_rad_ ||
        std::abs(measured[1] - head_recenter_seed_candidate_yaw_pitch_[1]) >
        head_recenter_seed_stability_rad_)
      {
        // A low or incorrect dq field must not make a changing physical pose
        // look stationary. Restart the consecutive position window at this
        // fresh sample and keep the SDK transport unopened.
        head_recenter_seed_candidate_yaw_pitch_ = measured;
        head_recenter_stable_seed_samples_ = 1;
        publish_debug_head({0.0, 0.0});
        return;
      } else {
        ++head_recenter_stable_seed_samples_;
      }
      if (head_recenter_stable_seed_samples_ < 3) {
        publish_debug_head({0.0, 0.0});
        return;
      }

      const TransportResult initialized = ensure_transport();
      if (!initialized.ok) {
        safe_stop_outputs(
          "head_recenter_transport_initialize_failed:" + initialized.detail, true);
        return;
      }
      const auto send_now = SteadyClock::now();
      const GateDecision send_gate = authorize_live_send(
        authorization_input(true, enabled_commands_fresh(send_now), true));
      if (!send_gate.allowed || !state_fresh(send_now) || !head_fresh(send_now)) {
        safe_stop_outputs(
          "head_recenter_post_initialize_gate_failed:" + send_gate.reason, true);
        return;
      }
      const std::array<double, 2> seed{
        latest_state_[kHeadYawIndex], latest_state_[kHeadPitchIndex]};
      if (!head_within_feedback_envelope(seed, head_absolute_limits_) ||
        !head_velocity_valid_ ||
        std::abs(latest_head_velocity_[0]) > max_head_seed_velocity_rad_s_ ||
        std::abs(latest_head_velocity_[1]) > max_head_seed_velocity_rad_s_)
      {
        safe_stop_outputs("head_recenter_seed_changed_or_unstable", true);
        return;
      }
      // Once seed_head crosses the publisher boundary, delivery can be
      // ambiguous even if the SDK reports an error. Mark cleanup ownership
      // first so fail-closed handling always asks the transport to relinquish.
      head_recenter_seed_ = latest_state_;
      const std::array<double, 2> clamped_seed =
        clamp_head_to_absolute_envelope(seed, head_absolute_limits_);
      held_head_target_ = head_recenter_seed_;
      held_head_target_[kHeadPitchIndex] = clamped_seed[1];
      held_head_target_[kHeadYawIndex] = clamped_seed[0];
      head_claimed_ = true;
      const TransportResult seeded = transport_->seed_head(held_head_target_);
      if (!seeded.ok) {
        safe_stop_outputs("head_recenter_seed_failed:" + seeded.detail, true);
        return;
      }
      head_recenter_target_yaw_pitch_ = clamped_seed;
      head_origin_yaw_pitch_ = clamped_seed;
      head_recenter_state_ = HeadRecenterState::LegacyMoving;
      head_recenter_started_time_ = send_now;
      head_recenter_last_advance_time_ = send_now;
      head_recenter_follow_wait_since_ = SteadyClock::time_point{};
      publish_debug_head(head_recenter_target_yaw_pitch_);
      publish_status(
        "head_auto_center=moving seed_yaw=" + std::to_string(seed[0]) +
        " seed_pitch=" + std::to_string(seed[1]) +
        " clamped_start_yaw=" + std::to_string(clamped_seed[0]) +
        " clamped_start_pitch=" + std::to_string(clamped_seed[1]), true);
      return;
    }

    const auto command_now = SteadyClock::now();
    const GateDecision command_gate = authorize_live_send(
      authorization_input(true, enabled_commands_fresh(command_now), true));
    if (!command_gate.allowed) {
      safe_stop_outputs("head_recenter_command_gate_failed:" + command_gate.reason, true);
      return;
    }

    const bool new_state = state_sequence_ != head_recenter_last_state_sequence_;
    if (new_state) {
      head_recenter_last_state_sequence_ = state_sequence_;
      // The commanded slew is deliberately slow.  A fresh feedback sample
      // that reports a materially faster physical motion indicates either a
      // tracking/controller fault or an external disturbance.  Stop before
      // publishing another target instead of relying only on position error.
      if (!head_velocity_valid_ ||
        std::abs(latest_head_velocity_[0]) >
        head_recenter_max_feedback_velocity_rad_s_ ||
        std::abs(latest_head_velocity_[1]) >
        head_recenter_max_feedback_velocity_rad_s_)
      {
        safe_stop_outputs("head_recenter_feedback_velocity_exceeded", true);
        return;
      }
    }

    if (head_recenter_state_ == HeadRecenterState::LegacyMoving && new_state) {
      const bool following =
        std::abs(measured[0] - head_recenter_target_yaw_pitch_[0]) <=
        head_recenter_follow_tolerance_rad_ &&
        std::abs(measured[1] - head_recenter_target_yaw_pitch_[1]) <=
        head_recenter_follow_tolerance_rad_;
      if (!following) {
        if (head_recenter_follow_wait_since_ == SteadyClock::time_point{}) {
          head_recenter_follow_wait_since_ = command_now;
        } else if (age(command_now, head_recenter_follow_wait_since_) >
          head_recenter_follow_timeout_sec_)
        {
          std::ostringstream detail;
          detail << "head_recenter_following_timeout"
                 << " feedback_yaw=" << measured[0]
                 << " feedback_pitch=" << measured[1]
                 << " target_yaw=" << head_recenter_target_yaw_pitch_[0]
                 << " target_pitch=" << head_recenter_target_yaw_pitch_[1]
                 << " error_yaw="
                 << (measured[0] - head_recenter_target_yaw_pitch_[0])
                 << " error_pitch="
                 << (measured[1] - head_recenter_target_yaw_pitch_[1])
                 << " wait_sec="
                 << age(command_now, head_recenter_follow_wait_since_);
          safe_stop_outputs(detail.str(), true);
          return;
        }
      } else {
        head_recenter_follow_wait_since_ = SteadyClock::time_point{};
        const double advance_dt = age(command_now, head_recenter_last_advance_time_);
        const auto next = step_head_toward_zero(
          head_recenter_target_yaw_pitch_, advance_dt,
          head_recenter_yaw_rate_rad_s_, head_recenter_pitch_rate_rad_s_);
        if (!finite_values(next) ||
          std::abs(next[0]) > std::abs(head_recenter_target_yaw_pitch_[0]) ||
          std::abs(next[1]) > std::abs(head_recenter_target_yaw_pitch_[1]) ||
          !head_within_absolute_limits(next, head_absolute_limits_))
        {
          safe_stop_outputs("head_recenter_nonmonotonic_or_invalid_target", true);
          return;
        }
        head_recenter_target_yaw_pitch_ = next;
        head_recenter_last_advance_time_ = command_now;
        if (next[0] == 0.0 && next[1] == 0.0) {
          head_recenter_state_ = HeadRecenterState::LegacyConfirming;
          head_recenter_zero_state_sequence_ = state_sequence_;
          head_recenter_confirmation_samples_ = 0;
          publish_status("head_recenter=confirming_zero_feedback", true);
        }
      }
    }

    // Every full-weight recenter trajectory frame is the exact 13-field seed
    // with only physical head pitch/yaw changed.
    held_head_target_ = head_recenter_seed_;
    held_head_target_[kHeadPitchIndex] = head_recenter_target_yaw_pitch_[1];
    held_head_target_[kHeadYawIndex] = head_recenter_target_yaw_pitch_[0];
    const TransportResult commanded = transport_->command_head(held_head_target_);
    if (!commanded.ok) {
      safe_stop_outputs("head_recenter_send_failed:" + commanded.detail, true);
      return;
    }
    publish_debug_head(head_recenter_target_yaw_pitch_);

    if (head_recenter_state_ == HeadRecenterState::LegacyConfirming && new_state &&
      state_sequence_ > head_recenter_zero_state_sequence_)
    {
      const bool confirmed =
        std::abs(measured[0]) <= head_recenter_tolerance_rad_ &&
        std::abs(measured[1]) <= head_recenter_tolerance_rad_ &&
        head_velocity_valid_ &&
        std::abs(latest_head_velocity_[0]) <= max_head_seed_velocity_rad_s_ &&
        std::abs(latest_head_velocity_[1]) <= max_head_seed_velocity_rad_s_;
      head_recenter_confirmation_samples_ = confirmed ?
        head_recenter_confirmation_samples_ + 1 : 0;
      head_recenter_zero_state_sequence_ = state_sequence_;
      if (head_recenter_confirmation_samples_ >= 5) {
        automatic_head_center_active_ = false;
        automatic_head_center_complete_ = true;
        head_recenter_state_ = HeadRecenterState::Disabled;
        head_origin_yaw_pitch_ = measured;
        last_head_yaw_pitch_ = {0.0, 0.0};
        held_head_target_[kHeadPitchIndex] = measured[1];
        held_head_target_[kHeadYawIndex] = measured[0];
        head_recenter_request_time_ = SteadyClock::time_point{};
        head_recenter_started_time_ = SteadyClock::time_point{};
        head_recenter_last_advance_time_ = SteadyClock::time_point{};
        head_recenter_follow_wait_since_ = SteadyClock::time_point{};
        publish_status(
          "head_auto_center=complete confirmed_samples=" +
          std::to_string(head_recenter_confirmation_samples_) +
          " control_handoff=vr_tracking", true);
      }
    }
  }

  void process_head(const SteadyClock::time_point & now, double dt)
  {
    if (!state_fresh(now) || !control_permission_fresh(now) ||
      !kill_clear_fresh(now))
    {
      publish_debug_head({0.0, 0.0});
      return;
    }
    if (!head_fresh(now)) {
      maintain_exhibition_arm_hold("head_command_stale");
      publish_debug_head(last_head_yaw_pitch_);
      return;
    }
    const auto shaped = clamp_head(latest_head_, last_head_yaw_pitch_, dt, head_limits_);
    publish_debug_head(shaped);
    if (!send_commands_) {
      last_head_yaw_pitch_ = shaped;
      return;
    }

    // Recompute command ages at the authorization point. In combined mode the
    // preceding head path may have waited briefly for the ArmSdk publisher;
    // the timer's earlier timestamp must not make an expired sample look fresh.
    const auto authorization_now = SteadyClock::now();
    const bool authorization_commands_fresh = exhibition_session_mode_ ?
      head_fresh(authorization_now) : enabled_commands_fresh(authorization_now);
    const GateDecision gate = authorize_live_send(
      authorization_input(
        true, authorization_commands_fresh, require_prepare_));
    if (!gate.allowed) {
      if (head_claimed_) {
        safe_stop_outputs("head_gate_failed:" + gate.reason, true);
      }
      return;
    }
    const TransportResult initialized = ensure_transport();
    if (!initialized.ok) {
      safe_stop_outputs("transport_initialize_failed:" + initialized.detail, true);
      return;
    }
    // SDK initialization is allowed only behind the same gates, but it may be
    // slow. Re-evaluate every freshness/interlock condition immediately before
    // the first ArmSdk publication (and on every already-initialized pass).
    const auto send_now = SteadyClock::now();
    const GateDecision send_gate = authorize_live_send(
      authorization_input(
        true,
        exhibition_session_mode_ ? head_fresh(send_now) :
        enabled_commands_fresh(send_now),
        require_prepare_));
    if (!send_gate.allowed) {
      safe_stop_outputs("head_post_initialize_gate_failed:" + send_gate.reason, true);
      return;
    }
    if (!head_claimed_) {
      const std::array<double, 2> seed{
        latest_state_[kHeadYawIndex], latest_state_[kHeadPitchIndex]};
      const bool seed_in_tracking_window =
        std::abs(seed[0]) <= max_head_tracking_seed_yaw_rad_ &&
        std::abs(seed[1]) <= max_head_tracking_seed_pitch_rad_;
      if (!seed_in_tracking_window && exhibition_session_mode_ &&
        !head_recenter_enabled_)
      {
        if (!head_within_feedback_envelope(seed, head_absolute_limits_)) {
          safe_stop_outputs("head_auto_center_seed_outside_feedback_envelope", true);
          return;
        }
        begin_automatic_exhibition_head_center(send_now, seed);
        return;
      }
      if (!head_within_absolute_limits(seed, head_absolute_limits_)) {
        std::ostringstream detail;
        detail << "head_seed_outside_confirmed_absolute_envelope"
               << " seed_yaw=" << seed[0]
               << " seed_pitch=" << seed[1]
               << " yaw_min=" << head_absolute_limits_.yaw_min
               << " yaw_max=" << head_absolute_limits_.yaw_max
               << " pitch_min=" << head_absolute_limits_.pitch_min
               << " pitch_max=" << head_absolute_limits_.pitch_max;
        safe_stop_outputs(detail.str(), true);
        return;
      }
      if (!seed_in_tracking_window)
      {
        std::ostringstream detail;
        detail << "head_seed_requires_explicit_recenter"
               << " seed_yaw=" << seed[0]
               << " seed_pitch=" << seed[1]
               << " tracking_seed_yaw_limit=" << max_head_tracking_seed_yaw_rad_
               << " tracking_seed_pitch_limit=" << max_head_tracking_seed_pitch_rad_;
        safe_stop_outputs(detail.str(), true);
        return;
      }
      // Start()/FSM 811 can finish its RPC/state transition just before the
      // firmware finishes the automatic head motion.  Never claim ArmSdk
      // ownership from that moving pose.  Wait for five *new* consecutive
      // LowState samples below the existing velocity ceiling, while retaining
      // a short fail-closed deadline.  No ArmSdk frame is published in this
      // settling phase.
      if (head_seed_settle_started_ == SteadyClock::time_point{}) {
        head_seed_settle_started_ = send_now;
        head_seed_settle_last_state_sequence_ = state_sequence_;
        head_seed_settle_stable_samples_ = 0;
        publish_status(
          "head_seed_waiting_for_stable_feedback required_samples=" +
          std::to_string(kHeadSeedStableSamples), true);
        return;
      }
      if (age(send_now, head_seed_settle_started_) > kHeadSeedSettleTimeoutSec) {
        std::ostringstream detail;
        detail << "head_seed_stable_feedback_timeout"
               << " stable_samples=" << head_seed_settle_stable_samples_
               << " required_samples=" << kHeadSeedStableSamples
               << " yaw_velocity=" << latest_head_velocity_[0]
               << " pitch_velocity=" << latest_head_velocity_[1]
               << " velocity_limit=" << max_head_seed_velocity_rad_s_;
        safe_stop_outputs(detail.str(), true);
        return;
      }
      if (state_sequence_ != head_seed_settle_last_state_sequence_) {
        head_seed_settle_last_state_sequence_ = state_sequence_;
        const bool stable = head_velocity_valid_ &&
          std::abs(latest_head_velocity_[0]) <= max_head_seed_velocity_rad_s_ &&
          std::abs(latest_head_velocity_[1]) <= max_head_seed_velocity_rad_s_;
        head_seed_settle_stable_samples_ = stable ?
          head_seed_settle_stable_samples_ + 1 : 0;
      }
      if (head_seed_settle_stable_samples_ < kHeadSeedStableSamples) {
        return;
      }
      reset_head_seed_settle();
      // Treat the first full-weight ArmSdk publication as potentially
      // delivered before entering it. A transport error is not proof that the
      // robot did not observe the frame, so safe-stop must still relinquish it.
      held_head_target_ = latest_state_;
      head_claimed_ = true;
      const TransportResult seeded = transport_->seed_head(held_head_target_);
      if (!seeded.ok) {
        safe_stop_outputs("head_seed_failed:" + seeded.detail, true);
        return;
      }
      head_origin_yaw_pitch_ = seed;
      head_tracking_seed_verified_ = true;
      last_head_yaw_pitch_ = {0.0, 0.0};
      std::ostringstream detail;
      detail << "head_seeded=all_13_fields_from_fresh_joint_state"
             << " head_tracking_seed_verified=true"
             << " origin_yaw=" << seed[0]
             << " origin_pitch=" << seed[1]
             << " relative_yaw_limit=" << head_limits_.yaw
             << " relative_pitch_limit=" << head_limits_.pitch;
      publish_status(detail.str(), true);
      return;
    }
    const auto absolute_target = absolute_head_target(
      head_origin_yaw_pitch_, shaped, head_absolute_limits_);
    if (!finite_values(absolute_target)) {
      safe_stop_outputs("head_absolute_target_invalid", true);
      return;
    }
    held_head_target_[kHeadPitchIndex] = absolute_target[1];
    held_head_target_[kHeadYawIndex] = absolute_target[0];
    const std::array<double, 2> effective_delta{
      absolute_target[0] - head_origin_yaw_pitch_[0],
      absolute_target[1] - head_origin_yaw_pitch_[1]};
    publish_debug_head(effective_delta);
    const TransportResult result = transport_->command_head(held_head_target_);
    if (!result.ok) {
      safe_stop_outputs("head_send_failed:" + result.detail, true);
      return;
    }
    // Track the delta that was actually commanded.  Keeping the unclamped
    // relative request here would wind up the limiter at an absolute stop and
    // cause a reversal dead-zone on the next inward request.
    last_head_yaw_pitch_ = effective_delta;
  }

  void process_arms(const SteadyClock::time_point & now, double dt)
  {
    if (!state_fresh(now) || !control_permission_fresh(now) ||
      !kill_clear_fresh(now))
    {
      publish_debug_arm({});
      return;
    }
    if (!arm_fresh(now)) {
      maintain_exhibition_arm_hold("arm_command_stale");
      publish_debug_arm(last_arm_command_);
      return;
    }
    if (!send_commands_) {
      const auto shaped = clamp_arm(
        latest_arm_, last_arm_command_, dt,
        arm_joint_delta_limit_rad_, arm_joint_rate_rad_s_);
      publish_debug_arm(shaped);
      last_arm_command_ = shaped;
      return;
    }

    const auto authorization_now = SteadyClock::now();
    const bool authorization_commands_fresh = exhibition_session_mode_ ?
      arm_fresh(authorization_now) : enabled_commands_fresh(authorization_now);
    const GateDecision gate = authorize_live_send(
      authorization_input(true, authorization_commands_fresh, require_prepare_));
    if (!gate.allowed) {
      if (head_claimed_) {
        safe_stop_outputs("arm_gate_failed:" + gate.reason, true);
      }
      return;
    }
    const TransportResult initialized = ensure_transport();
    if (!initialized.ok) {
      safe_stop_outputs("arm_transport_initialize_failed:" + initialized.detail, true);
      return;
    }
    const auto send_now = SteadyClock::now();
    const GateDecision send_gate = authorize_live_send(
      authorization_input(
        true,
        exhibition_session_mode_ ? arm_fresh(send_now) :
        enabled_commands_fresh(send_now),
        require_prepare_));
    if (!send_gate.allowed) {
      safe_stop_outputs("arm_post_initialize_gate_failed:" + send_gate.reason, true);
      return;
    }

    if (!head_claimed_) {
      held_head_target_ = latest_state_;
      head_claimed_ = true;
      const TransportResult seeded = transport_->seed_head(held_head_target_);
      if (!seeded.ok) {
        safe_stop_outputs("arm_seed_failed:" + seeded.detail, true);
        return;
      }
      for (std::size_t index = 0; index < 10; ++index) {
        last_arm_command_[index] = held_head_target_[index];
      }
      arm_neutral_hold_.arm(
        std::array<double, 10>{
          held_head_target_[0], held_head_target_[1], held_head_target_[2],
          held_head_target_[3], held_head_target_[4], held_head_target_[5],
          held_head_target_[6], held_head_target_[7], held_head_target_[8],
          held_head_target_[9]});
      reset_arm_feedback_follow(held_head_target_);
      publish_status(
        "arm_seeded=all_13_fields_from_fresh_joint_state "
        "arm_neutral_hold=awaiting_first_vr_target", true);
      return;
    }

    if (!arm_neutral_hold_.active()) {
      arm_neutral_hold_.arm(
        std::array<double, 10>{
          held_head_target_[0], held_head_target_[1], held_head_target_[2],
          held_head_target_[3], held_head_target_[4], held_head_target_[5],
          held_head_target_[6], held_head_target_[7], held_head_target_[8],
          held_head_target_[9]});
    }
    if (!arm_neutral_hold_.neutral_captured()) {
      // The first target after a stop/kill may already contain the operator's
      // old controller offset. Capture it as the new neutral and keep the
      // exact physical feedback seed; never send that stale displacement.
      if (arm_neutral_hold_.capture_first_target(latest_arm_)) {
        // Head control may already own the shared frame, bypassing the arm
        // seed branch above. Seed response references before either path sends
        // its first arm displacement; never seed from that later moving target.
        reset_arm_feedback_follow(held_head_target_);
        publish_debug_arm(last_arm_command_);
        publish_status("arm_neutral_hold=captured_first_vr_target", true);
      }
      return;
    }

    const auto neutralized_target = arm_targets_absolute_ ?
      arm_neutral_hold_.target_for(latest_arm_, true) : arm_neutral_hold_.target_for(latest_arm_);
    const auto shaped = clamp_arm(
      neutralized_target, last_arm_command_, dt,
      arm_joint_delta_limit_rad_, arm_joint_rate_rad_s_);
    publish_debug_arm(shaped);

    for (std::size_t index = 0; index < 10; ++index) {
      held_head_target_[index] = shaped[index];
    }
    const TransportResult result = transport_->command_head(held_head_target_);
    if (!result.ok) {
      safe_stop_outputs("arm_send_failed:" + result.detail, true);
      return;
    }
    last_arm_command_ = shaped;
    record_arm_feedback_follow_target(held_head_target_, SteadyClock::now());
  }

  // ArmSdk hands the frame to a publisher thread, so a successful local send
  // does not prove that the R1 accepted authority.  Once a meaningful target
  // has been queued, require fresh LowState movement in the same direction.
  // This catches Debug mode, unavailable arm authority, and a silently
  // ignored publisher stream before an operator keeps trying commands.
  void reset_arm_feedback_follow(const ArmSdkPositions & baseline)
  {
    ArmResponseGuard::Positions seed{};
    std::copy_n(baseline.begin(), seed.size(), seed.begin());
    arm_response_guard_.seed(seed);
  }

  void clear_arm_feedback_follow()
  {
    arm_response_guard_.clear();
  }

  void record_arm_feedback_follow_target(
    const ArmSdkPositions & target, const SteadyClock::time_point & now)
  {
    if (!(send_commands_ && transport_name_ == "sdk" && enable_arms_)) {
      return;
    }

    ArmResponseGuard::Positions command{}, feedback{};
    std::copy_n(target.begin(), command.size(), command.begin());
    std::copy_n(latest_state_.begin(), feedback.size(), feedback.begin());
    arm_response_guard_.record_target(
      command, feedback, state_sequence_,
      std::chrono::duration<double>(now.time_since_epoch()).count());
  }

  std::string arm_feedback_follow_failure(const SteadyClock::time_point & now)
  {
    if (!(send_commands_ && transport_name_ == "sdk" && enable_arms_) ||
      !arm_response_guard_.pending() ||
      !state_fresh(now))
    {
      return "";
    }
    ArmResponseGuard::Positions feedback{};
    std::copy_n(latest_state_.begin(), feedback.size(), feedback.begin());
    const auto failure = arm_response_guard_.observe(
      feedback, state_sequence_,
      std::chrono::duration<double>(now.time_since_epoch()).count());
    if (!failure) {
      return "";
    }
    std::ostringstream detail;
    detail << "arm_feedback_follow_timeout joint="
           << kArmSdkJointNames[failure->joint]
           << " requested_direction=" << failure->direction
           << " target=" << failure->target
           << " observation_target=" << failure->observation_target
           << " command_reference=" << failure->command_reference
           << " feedback=" << failure->feedback
           << " feedback_baseline=" << failure->feedback_baseline
           << " feedback_progress=" << failure->feedback - failure->feedback_baseline
           << " required_progress=" << arm_feedback_follow_min_progress_rad_
           << " progress_samples=" << failure->progress_samples
           << " required_samples=" << ArmResponseGuard::kRequiredProgressSamples
           << " elapsed_sec=" << failure->elapsed_sec
           << " timeout_sec=" << arm_feedback_follow_timeout_sec_;
    return detail.str();
  }

  void process_velocity(const SteadyClock::time_point & now, double dt)
  {
    if (!state_fresh(now) || !control_permission_fresh(now) ||
      !kill_clear_fresh(now))
    {
      publish_debug_velocity({0.0, 0.0, 0.0});
      return;
    }
    if (!velocity_fresh(now)) {
      maintain_exhibition_velocity_zero("velocity_command_stale");
      return;
    }
    auto shaped = clamp_velocity(
      latest_velocity_, last_velocity_command_, dt, velocity_limits_);
    if (velocity_status_127_probe_enabled_) {
      // The one-shot compatibility experiment is forward-only. Ignore stick
      // noise on lateral/yaw and never permit reverse motion in this mode.
      shaped = {
        std::clamp(shaped[0], 0.0, kVelocity127ProbeMaxForwardMps),
        0.0,
        0.0};
    }
    publish_debug_velocity(shaped);
    if (!send_commands_) {
      last_velocity_command_ = shaped;
      return;
    }

    const auto authorization_now = SteadyClock::now();
    const bool authorization_commands_fresh = exhibition_session_mode_ ?
      velocity_fresh(authorization_now) : enabled_commands_fresh(authorization_now);
    const GateDecision gate = authorize_live_send(
      authorization_input(
        true, authorization_commands_fresh, require_prepare_));
    if (!gate.allowed) {
      if (locomotion_active_) {
        safe_stop_outputs("locomotion_gate_failed:" + gate.reason, true);
      }
      return;
    }
    const TransportResult initialized = ensure_transport();
    if (!initialized.ok) {
      safe_stop_outputs("transport_initialize_failed:" + initialized.detail, true);
      return;
    }
    const auto send_now = SteadyClock::now();
    const GateDecision send_gate = authorize_live_send(
      authorization_input(
        true,
        exhibition_session_mode_ ? velocity_fresh(send_now) :
        enabled_commands_fresh(send_now),
        require_prepare_));
    if (!send_gate.allowed) {
      safe_stop_outputs("locomotion_post_initialize_gate_failed:" + send_gate.reason, true);
      return;
    }
    if (!is_zero(shaped) && velocity_status_127_probe_enabled_ &&
      velocity_127_probe_started_ != SteadyClock::time_point{} &&
      age(send_now, velocity_127_probe_started_) >= kVelocity127ProbeMaxDurationSec)
    {
      safe_stop_outputs("velocity_127_probe_window_complete", true);
      return;
    }
    if (is_zero(shaped)) {
      if (locomotion_active_) {
        if (exhibition_session_mode_) {
          maintain_exhibition_velocity_zero("fresh_zero_command");
        } else {
          const TransportResult result = transport_->stop_locomotion();
          if (!result.ok) {
            safe_stop_outputs("locomotion_stop_failed:" + result.detail, true);
            return;
          }
          locomotion_active_ = false;
        }
      }
      last_velocity_command_ = {0.0, 0.0, 0.0};
      return;
    }
    const double locomotion_command_rate_hz =
      locomotion_command_mode_ == "wireless_controller" ?
      wireless_controller_rate_hz_ : locomotion_rpc_rate_hz_;
    const double command_period_sec = 1.0 / locomotion_command_rate_hz;
    if (last_velocity_rpc_send_ != SteadyClock::time_point{} &&
      age(send_now, last_velocity_rpc_send_) < command_period_sec)
    {
      // Continue shaping at the 100 Hz safety-loop rate, but refresh the
      // locomotion command only at the bounded transport-specific rate.
      last_velocity_command_ = shaped;
      return;
    }
    const bool first_command_for_motion_interval = !locomotion_active_;
    if (velocity_status_127_probe_enabled_ &&
      velocity_127_probe_started_ == SteadyClock::time_point{})
    {
      velocity_127_probe_started_ = send_now;
      publish_status(
        "velocity_127_probe=started max_duration_sec=" +
        std::to_string(kVelocity127ProbeMaxDurationSec) +
        " max_forward_mps=" + std::to_string(kVelocity127ProbeMaxForwardMps),
        true);
    }
    if (first_command_for_motion_interval) {
      RCLCPP_INFO(
        get_logger(),
        "Locomotion command attempt mode=%s forward=%.3f lateral=%.3f "
        "yaw=%.3f duration=%.3f",
        locomotion_command_mode_.c_str(),
        shaped[0], shaped[1], shaped[2], command_duration_sec_);
    }
    // Crossing a new locomotion boundary creates fresh cleanup debt inside
    // the SDK transport even when delivery later returns an ambiguous error.
    // Do not let a recent stop retry from an older interval defer the
    // immediate cleanup attempt for this new command.
    last_locomotion_cleanup_attempt_ = SteadyClock::time_point{};
    const TransportResult result = transport_->set_velocity(
      shaped[0], shaped[1], shaped[2], command_duration_sec_);
    if (!result.ok) {
      if (velocity_status_127_probe_enabled_ && result.status_code == 127) {
        if (first_command_for_motion_interval) {
          RCLCPP_WARN(
            get_logger(),
            "SetVelocity status 127 admitted only by bounded probe; "
            "continuing at %.1f Hz for at most %.1f s",
            locomotion_rpc_rate_hz_, kVelocity127ProbeMaxDurationSec);
        }
        locomotion_active_ = true;
        last_velocity_rpc_send_ = SteadyClock::now();
        last_velocity_command_ = shaped;
        return;
      }
      std::ostringstream failure_detail;
      failure_detail << "Locomotion_command_failed mode=" <<
        locomotion_command_mode_ << " forward=" << shaped[0]
                     << " lateral=" << shaped[1]
                     << " yaw=" << shaped[2]
                     << " duration=" << command_duration_sec_
                     << ":" << result.detail;
      safe_stop_outputs(failure_detail.str(), true);
      return;
    }
    if (first_command_for_motion_interval) {
      RCLCPP_INFO(
        get_logger(), "Locomotion command result ok=true detail=%s",
        result.detail.c_str());
    }
    locomotion_active_ = true;
    last_velocity_rpc_send_ = SteadyClock::now();
    last_velocity_command_ = shaped;
  }

  AuthorizationInput authorization_input(
    bool feature_enabled, bool command_fresh, bool require_prepared) const
  {
    const auto now = SteadyClock::now();
    const LiveEnvironment environment = LiveEnvironment::from_process();
    AuthorizationInput result;
    result.send_commands = send_commands_;
    result.feature_enabled = feature_enabled;
    result.commissioning_parameter_confirmed = commissioning_confirmed_;
    result.commissioning_token_matches = commissioning_parameter_ok(environment);
    result.require_vr_source = !static_prepare_mode_;
    result.vr_source_matches = control_source_ok(environment);
    result.profile_is_slow_safe = profile_ == "slow-safe";
    result.kill_clear_fresh = kill_clear_fresh(now);
    result.deadman_active_fresh =
      static_prepare_mode_ || control_permission_fresh(now);
    result.state_fresh = state_fresh(now);
    result.motor_health_required = motor_health_required();
    result.motors_healthy_fresh = motor_health_fresh(now);
    result.command_fresh = command_fresh;
    result.prepared = prepared_ && prepare_rearm_gate_.ready();
    result.require_prepared = require_prepared;
    result.environment = environment;
    return result;
  }

  bool commissioning_parameter_ok(const LiveEnvironment & environment) const
  {
    return commissioning_confirmed_ && commissioning_token_.size() >= 16 &&
           environment.commissioning_token == commissioning_token_;
  }

  bool head_recenter_environment_confirmed() const
  {
    const char * const value = std::getenv("ROBOT_CONFIRM_HEAD_RECENTER");
    return value != nullptr && std::string(value) == "1";
  }

  bool head_ownership_probe_environment_confirmed() const
  {
    const char * const value = std::getenv("ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE");
    return value != nullptr && std::string(value) == "1";
  }

  bool vr_source_ok(const LiveEnvironment & environment) const
  {
    return environment.vr_transport == vr_transport_ &&
           valid_vr_source(expected_vr_source_ip_, vr_transport_) &&
           valid_vr_source(environment.vr_source_ip, environment.vr_transport) &&
           environment.vr_source_ip == expected_vr_source_ip_;
  }

  bool exhibition_session_environment_ok() const
  {
    if (!exhibition_session_mode_) {
      return true;
    }
    const char * const value = std::getenv("ROBOT_EXHIBITION_SESSION");
    return value != nullptr && std::string(value) == "1";
  }

  bool static_prepare_environment_ok() const
  {
    const char * const value = std::getenv("ROBOT_CONFIRM_STATIC_PREPARE");
    return value != nullptr && std::string(value) == "1";
  }

  bool velocity_127_probe_environment_confirmed() const
  {
    const char * const probe = std::getenv("ROBOT_CONFIRM_VELOCITY_127_PROBE");
    const char * const app_closed =
      std::getenv("ROBOT_CONFIRM_UNITREE_EXPLORE_CLOSED");
    const char * const no_phone =
      std::getenv("ROBOT_CONFIRM_NO_PHONE_CONTROL");
    const char * const firmware =
      std::getenv("ROBOT_CONFIRM_AI_SPORT_1_0_2_154");
    return probe != nullptr && std::string(probe) == "1" &&
           ((app_closed != nullptr && std::string(app_closed) == "1") ||
           (no_phone != nullptr && std::string(no_phone) == "1")) &&
           firmware != nullptr && std::string(firmware) == "1";
  }

  bool live_environment_complete(const LiveEnvironment & environment) const
  {
    return environment.missing(!static_prepare_mode_).empty();
  }

  bool control_source_ok(const LiveEnvironment & environment) const
  {
    if (static_prepare_mode_) {
      return static_prepare_environment_ok();
    }
    return vr_source_ok(environment) && exhibition_session_environment_ok();
  }

  bool control_permission_fresh(const SteadyClock::time_point & now) const
  {
    if (static_prepare_mode_) {
      return static_prepare_environment_ok();
    } else if (exhibition_session_mode_) {
      return session_control_seen_ && session_control_armed_ &&
             exhibition_session_environment_ok();
    }
    return deadman_fresh(now);
  }

  TransportResult ensure_transport()
  {
    if (!send_commands_) {
      return {false, "send_commands=false; transport initialization forbidden"};
    }
    if (transport_->initialized()) {
      return {true, "transport already initialized"};
    }
    // The caller has just evaluated the complete per-send gate.  Re-evaluate
    // its common pieces immediately before crossing the sole SDK boundary.
    const AuthorizationInput gate_input = authorization_input(
      enable_head_ || enable_arms_ || enable_locomotion_ || enable_prepare_, true, false);
    const GateDecision gate = authorize_live_send(gate_input);
    if (!gate.allowed) {
      return {false, "transport initialization blocked: " + gate.reason};
    }
    return transport_->initialize(
      network_interface_, enable_head_ || enable_arms_, enable_locomotion_ || enable_prepare_);
  }

  std::string active_watchdog_failure(const SteadyClock::time_point & now) const
  {
    if (!kill_clear_fresh(now)) {
      return "watchdog_kill_missing_active_or_stale";
    }
    if (static_prepare_mode_) {
      // A featureless static writer never reaches the active output loop.
    } else if (exhibition_session_mode_) {
      if (!control_permission_fresh(now)) {
        return "watchdog_exhibition_session_disarmed";
      }
    } else if (!deadman_fresh(now)) {
      return "watchdog_deadman_released_or_stale";
    }
    if (!state_fresh(now)) {
      return "watchdog_joint_state_missing_stale_or_invalid";
    }
    if (!motor_health_fresh(now)) {
      return "watchdog_motor_health_missing_stale_or_unhealthy";
    }
    if (!exhibition_session_mode_) {
      if (enable_head_ && !head_fresh(now)) {
        return "watchdog_head_command_missing_stale_or_invalid";
      }
      if (enable_arms_ && !arm_fresh(now)) {
        return "watchdog_arm_command_missing_stale_or_invalid";
      }
      if (enable_locomotion_ && !velocity_fresh(now)) {
        return "watchdog_velocity_missing_stale_or_invalid";
      }
    }
    const LiveEnvironment environment = LiveEnvironment::from_process();
    if (!live_environment_complete(environment) ||
      !commissioning_parameter_ok(environment) ||
      !control_source_ok(environment) ||
      profile_ != "slow-safe")
    {
      return "watchdog_live_interlock_changed";
    }
    if (head_recenter_enabled_ &&
      (!head_ownership_probe_confirmed_ ||
      !head_ownership_probe_environment_confirmed()))
    {
      return "watchdog_head_ownership_probe_confirmation_lost";
    }
    if (velocity_status_127_probe_enabled_ &&
      !velocity_127_probe_environment_confirmed())
    {
      return "watchdog_velocity_127_probe_confirmation_lost";
    }
    return {};
  }

  std::string prepare_watchdog_failure(const SteadyClock::time_point & now) const
  {
    if (!kill_clear_fresh(now)) {
      return "watchdog_kill_missing_active_or_stale";
    }
    if (static_prepare_mode_) {
      // No Deadman/session input exists in the featureless static graph.
    } else if (exhibition_session_mode_) {
      if (!control_permission_fresh(now)) {
        return "watchdog_exhibition_session_disarmed";
      }
    } else if (!deadman_fresh(now)) {
      return "watchdog_deadman_released_or_stale";
    }
    if (!state_fresh(now)) {
      return "watchdog_joint_state_missing_stale_or_invalid";
    }
    if (!motor_health_fresh(now)) {
      return "watchdog_motor_health_missing_stale_or_unhealthy";
    }
    const LiveEnvironment environment = LiveEnvironment::from_process();
    if (!live_environment_complete(environment) ||
      !commissioning_parameter_ok(environment) ||
      !control_source_ok(environment) ||
      profile_ != "slow-safe")
    {
      return "watchdog_live_interlock_changed";
    }
    if (velocity_status_127_probe_enabled_ &&
      !velocity_127_probe_environment_confirmed())
    {
      return "watchdog_velocity_127_probe_confirmation_lost";
    }
    return {};
  }

  void finish_prepare_if_ready()
  {
    if (!prepare_in_progress_ || !prepare_future_.valid() ||
      prepare_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      return;
    }

    TransportResult result;
    try {
      result = prepare_future_.get();
    } catch (const std::exception & exception) {
      result = {false, std::string("prepare worker exception: ") + exception.what()};
    } catch (...) {
      result = {false, "prepare worker unknown exception"};
    }
    prepare_in_progress_ = false;
    const bool cancelled = prepare_cancel_requested_.exchange(false);
    if (cancelled) {
      // The first safe-stop request deliberately deferred SDK access while the
      // worker owned LocoClient.  Now that the future is joined, perform the
      // owed StopMove/release cleanup without any concurrent SDK call.  Do not
      // change the original request's global-kill policy; the local latch is
      // already authoritative and remains asserted.
      safe_stop_outputs("prepare_cancelled result=" + result.detail, true, false);
      return;
    }
    if (!result.ok) {
      safe_stop_outputs("prepare_failed:" + result.detail, true);
      return;
    }

    // Inputs could change during the bounded SDK polling interval.  Require
    // the complete common live gate again before accepting its result.
    const GateDecision gate = authorize_live_send(
      authorization_input(enable_prepare_, true, false));
    if (!gate.allowed) {
      safe_stop_outputs("prepare_completion_gate_failed:" + gate.reason, true);
      return;
    }
    prepared_ = true;
    if (static_prepare_mode_) {
      (void)prepare_rearm_gate_.prepared_session(
        false, std::array<double, 3>{0.0, 0.0, 0.0});
    } else if (exhibition_session_mode_) {
      const auto now = SteadyClock::now();
      if (enable_locomotion_ && !velocity_fresh(now)) {
        safe_stop_outputs(
          "prepare_session_rearm_failed:neutral_velocity_missing_or_stale", true);
        return;
      }
      if (!prepare_rearm_gate_.prepared_session(
          enable_locomotion_, enable_locomotion_ ? latest_velocity_ :
          std::array<double, 3>{0.0, 0.0, 0.0}))
      {
        safe_stop_outputs(
          "prepare_session_rearm_failed:sticks_not_neutral", true);
        return;
      }
    } else {
      prepare_rearm_gate_.prepared(enable_locomotion_);
    }
    // Commands used to authorize StandUp are deliberately not reusable. The
    // normal mode still requires a physical release/press edge; exhibition
    // mode has already passed its explicit session-arm + neutral-stick gate.
    clear_command_freshness();
    publish_armed();
    publish_status(
      "prepare_result=" + result.detail +
      " prepare_rearm=" + prepare_rearm_gate_.state_name(), true);
  }

  bool enabled_commands_fresh(const SteadyClock::time_point & now) const
  {
    // In combined mode this is an atomic readiness barrier: neither SDK
    // output can be activated from a one-sided or pre-re-arm command sample.
    return (!enable_head_ || head_fresh(now)) &&
           (!enable_arms_ || arm_fresh(now)) &&
           (!enable_locomotion_ || velocity_fresh(now));
  }

  void clear_command_freshness()
  {
    head_valid_ = false;
    arm_valid_ = false;
    velocity_valid_ = false;
    head_arrival_ = SteadyClock::time_point{};
    arm_arrival_ = SteadyClock::time_point{};
    velocity_arrival_ = SteadyClock::time_point{};
    arm_neutral_hold_.reset();
  }

  void safe_stop_outputs(
    const std::string & reason, bool latch_kill, bool request_global_kill = true)
  {
    const auto cleanup_now = SteadyClock::now();
    const bool force_cleanup_attempt =
      reason == "operator_stop" || reason == "operator_kill_service" ||
      reason == "node_shutdown";
    const bool locomotion_retry_due =
      force_cleanup_attempt ||
      last_locomotion_cleanup_attempt_ == SteadyClock::time_point{} ||
      age(cleanup_now, last_locomotion_cleanup_attempt_) >= kCleanupRetryIntervalSec;
    const bool status_retry_due =
      force_cleanup_attempt ||
      last_safe_stop_status_time_ == SteadyClock::time_point{} ||
      age(cleanup_now, last_safe_stop_status_time_) >= kCleanupRetryIntervalSec;
    if (reason.rfind("head_ownership_probe_failed", 0) == 0) {
      head_probe_terminal_status_ =
        "fail_closed reason=" + reason + " terminal=true";
    }
    std::string detail;
    bool locomotion_cleanup_pending = false;
    bool head_cleanup_pending = false;
    if (prepare_in_progress_) {
      prepare_cancel_requested_.store(true);
      detail += " prepare_cancel=requested";
    }
    if (send_commands_ && transport_ && transport_->initialized() &&
      !prepare_in_progress_)
    {
      // StandUp/SetVelocity may be delivered before their RPC result is known.
      // Ask the transport to stop even when the node has not yet marked the
      // command active; the transport itself tracks whether StopMove is due.
      if (enable_locomotion_ || enable_prepare_) {
        if (locomotion_retry_due) {
          last_locomotion_cleanup_attempt_ = cleanup_now;
          const TransportResult stopped = transport_->stop_locomotion();
          detail += " StopMove=" + stopped.detail;
          locomotion_cleanup_pending = !stopped.ok;
        } else {
          // A failed/ambiguous StopMove keeps the debt asserted, but retrying
          // it at the 100 Hz safety-loop rate floods both the DDS service and
          // the log. Preserve the debt and retry at a bounded 2 Hz instead.
          locomotion_cleanup_pending = locomotion_active_;
          if (locomotion_cleanup_pending) {
            detail += " StopMove=retry_deferred";
          }
        }
      }
      if (head_claimed_) {
        const auto now = SteadyClock::now();
        ArmSdkPositions safe_pose = state_fresh(now) ?
          latest_state_ : held_head_target_;
        const auto safe_head = clamp_head_to_absolute_envelope(
          {safe_pose[kHeadYawIndex], safe_pose[kHeadPitchIndex]},
          head_absolute_limits_);
        safe_pose[kHeadPitchIndex] = safe_head[1];
        safe_pose[kHeadYawIndex] = safe_head[0];
        // The transport hold contract preserves its last confirmed/requested
        // ownership weight, and release either descends from that weight or
        // publishes zero directly after an ambiguous frame. Cleanup must never
        // convert a passive/partial probe claim into a stronger claim.
        const TransportResult held = transport_->hold_head(safe_pose);
        detail += " head_hold=" + held.detail;
        const TransportResult released = transport_->release_head(safe_pose);
        detail += " head_release=" + released.detail;
        head_cleanup_pending = !released.ok;
      }
    }
    // Never report an ambiguous/failed cleanup as inactive.  Keeping these
    // flags asserted makes the watchdog retry on the next tick and prevents
    // reset/prepare from proceeding until StopMove/release actually succeeds.
    locomotion_active_ = locomotion_cleanup_pending;
    head_claimed_ = head_cleanup_pending;
    last_safe_stop_detail_ = detail;
    prepared_ = false;
    sequential_control_.reset();
    sequential_zero_sent_ = SteadyClock::time_point{};
    prepare_rearm_gate_.reset();
    reset_head_recenter_state();
    reset_head_seed_settle();
    clear_arm_feedback_follow();
    clear_command_freshness();
    exhibition_hold_active_ = false;
    exhibition_dropout_started_ = SteadyClock::time_point{};
    velocity_127_probe_started_ = SteadyClock::time_point{};
    last_velocity_command_ = {0.0, 0.0, 0.0};
    last_velocity_rpc_send_ = SteadyClock::time_point{};
    if (state_valid_) {
      head_origin_yaw_pitch_ = {
        latest_state_[kHeadYawIndex], latest_state_[kHeadPitchIndex]};
    }
    last_head_yaw_pitch_ = {0.0, 0.0};
    if (latch_kill) {
      const bool newly_latched = !local_kill_latched_;
      local_kill_latched_ = true;
      if (request_global_kill && newly_latched) {
        publish_kill_request(true);
      }
    }
    publish_debug_velocity({0.0, 0.0, 0.0});
    publish_debug_arm({});
    publish_debug_head({0.0, 0.0});
    publish_armed();
    if (status_retry_due) {
      last_safe_stop_status_time_ = cleanup_now;
      publish_status(
        "fail_closed reason=" + reason + " kill_latched=" + bool_text(latch_kill) + detail,
        true);
    }
  }

  void publish_kill_request(bool asserted = true)
  {
    std_msgs::msg::Bool request;
    request.data = asserted;
    kill_request_publisher_->publish(request);
  }

  bool outputs_active() const
  {
    return locomotion_active_ || head_claimed_ || head_recenter_requested();
  }

  void reset_head_seed_settle()
  {
    head_tracking_seed_verified_ = false;
    head_seed_settle_started_ = SteadyClock::time_point{};
    head_seed_settle_last_state_sequence_ = 0;
    head_seed_settle_stable_samples_ = 0;
  }

  bool state_fresh(const SteadyClock::time_point & now) const
  {
    return state_valid_ && state_arrival_ != SteadyClock::time_point{} &&
           age(now, state_arrival_) <= state_timeout_sec_;
  }

  bool motor_health_required() const
  {
    return transport_name_ == "sdk" && send_commands_;
  }

  bool motor_health_fresh(const SteadyClock::time_point & now) const
  {
    if (!motor_health_required()) {
      return true;
    }
    return motor_health_seen_ && motors_healthy_ &&
           motor_health_arrival_ != SteadyClock::time_point{} &&
           age(now, motor_health_arrival_) <= motor_health_timeout_sec_;
  }

  bool head_fresh(const SteadyClock::time_point & now) const
  {
    return head_valid_ && head_arrival_ != SteadyClock::time_point{} &&
           age(now, head_arrival_) <= command_timeout_sec_;
  }

  bool arm_fresh(const SteadyClock::time_point & now) const
  {
    return arm_valid_ && arm_arrival_ != SteadyClock::time_point{} &&
           age(now, arm_arrival_) <= command_timeout_sec_;
  }

  bool velocity_fresh(const SteadyClock::time_point & now) const
  {
    return velocity_valid_ && velocity_arrival_ != SteadyClock::time_point{} &&
           age(now, velocity_arrival_) <= command_timeout_sec_;
  }

  bool deadman_fresh(const SteadyClock::time_point & now) const
  {
    return deadman_seen_ && deadman_active_ &&
           deadman_arrival_ != SteadyClock::time_point{} &&
           age(now, deadman_arrival_) <= deadman_timeout_sec_;
  }

  bool kill_clear_fresh(const SteadyClock::time_point & now) const
  {
    return kill_seen_ && !kill_signal_active_ && !local_kill_latched_ &&
           kill_arrival_ != SteadyClock::time_point{} &&
           age(now, kill_arrival_) <= kill_timeout_sec_;
  }

  static double age(
    const SteadyClock::time_point & now, const SteadyClock::time_point & then)
  {
    if (then == SteadyClock::time_point{}) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(now - then).count();
  }

  void publish_debug_head(const std::array<double, 2> & yaw_pitch)
  {
    trajectory_msgs::msg::JointTrajectory message;
    message.header.stamp = get_clock()->now();
    message.joint_names = {"head_yaw_joint", "head_pitch_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {yaw_pitch[0], yaw_pitch[1]};
    message.points.push_back(point);
    debug_head_publisher_->publish(message);
  }

  void publish_debug_arm(const std::array<double, 10> & positions)
  {
    trajectory_msgs::msg::JointTrajectory message;
    message.header.stamp = get_clock()->now();
    message.joint_names.assign(kArmSdkJointNames.begin(), kArmSdkJointNames.begin() + 10);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(positions.begin(), positions.end());
    message.points.push_back(point);
    debug_arm_publisher_->publish(message);
  }

  void publish_debug_velocity(const std::array<double, 3> & velocity)
  {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "base_link";
    message.twist.linear.x = velocity[0];
    message.twist.linear.y = velocity[1];
    message.twist.angular.z = velocity[2];
    debug_velocity_publisher_->publish(message);
  }

  void publish_armed()
  {
    const auto now = SteadyClock::now();
    const LiveEnvironment environment = LiveEnvironment::from_process();
    const bool default_deadman_state_and_health =
      deadman_fresh(now) && state_fresh(now) && motor_health_fresh(now);
    const bool permission_state_and_health = static_prepare_mode_ ?
      (state_fresh(now) && motor_health_fresh(now)) :
      (exhibition_session_mode_ ?
      (control_permission_fresh(now) && state_fresh(now) && motor_health_fresh(now)) :
      default_deadman_state_and_health);
    std_msgs::msg::Bool message;
    message.data = send_commands_ && transport_ && transport_->initialized() &&
      !prepare_in_progress_ &&
      !local_kill_latched_ && kill_clear_fresh(now) &&
      permission_state_and_health &&
      enabled_commands_fresh(now) &&
      live_environment_complete(environment) && commissioning_parameter_ok(environment) &&
      control_source_ok(environment) &&
      profile_ == "slow-safe" &&
      (!require_prepare_ || (prepared_ && prepare_rearm_gate_.ready())) &&
      (!head_recenter_enabled_ || head_recenter_requested()) &&
      !automatic_head_center_active_;
    armed_publisher_->publish(message);
  }

  void publish_periodic_status(const SteadyClock::time_point & now)
  {
    if (last_status_time_ != SteadyClock::time_point{} &&
      age(now, last_status_time_) < 1.0)
    {
      return;
    }
    const LiveEnvironment environment = LiveEnvironment::from_process();
    std::string text =
      "transport=" + transport_name_ +
      " arm_sdk_frame_profile=" + arm_sdk_frame_profile_ +
      " initialized=" + bool_text(transport_ && transport_->initialized()) +
      " sequential_control=" + bool_text(sequential_control_enabled_) +
      " sequential_phase=" + std::string(sequential_control_.name()) +
      " send_commands=" + bool_text(send_commands_) +
      " prepare_in_progress=" + bool_text(prepare_in_progress_) +
      " prepared=" + bool_text(prepared_) +
      " prepare_rearm=" + prepare_rearm_gate_.state_name() +
      " head_claimed=" + bool_text(head_claimed_) +
      " head_recenter=" + std::string(head_recenter_state_name(head_recenter_state_)) +
      " head_auto_center_active=" + bool_text(automatic_head_center_active_) +
      " head_auto_center_complete=" + bool_text(automatic_head_center_complete_) +
      " head_tracking_seed_verified=" + bool_text(head_tracking_seed_verified_) +
      " head_auto_center_paused=" + bool_text(automatic_head_center_paused_) +
      " head_recenter_confirm_samples=" +
      std::to_string(head_recenter_confirmation_samples_) +
      " locomotion_active=" + bool_text(locomotion_active_) +
      " static_prepare_mode=" + bool_text(static_prepare_mode_) +
      " exhibition_session_mode=" + bool_text(exhibition_session_mode_) +
      " session_armed=" + bool_text(session_control_seen_ && session_control_armed_) +
      " vr_recovery=" + std::string(exhibition_hold_active_ ? "hold" : "tracking") +
      " kill_clear=" + bool_text(kill_clear_fresh(now)) +
      " deadman_fresh=" + bool_text(deadman_fresh(now)) +
      " control_permission=" + bool_text(control_permission_fresh(now)) +
      " state_fresh=" + bool_text(state_fresh(now)) +
      " motor_health_fresh=" + bool_text(motor_health_fresh(now)) +
      " head_fresh=" + bool_text(head_fresh(now)) +
      " arm_fresh=" + bool_text(arm_fresh(now)) +
      " velocity_fresh=" + bool_text(velocity_fresh(now)) +
      " arm_feedback_follow_pending=" + bool_text(arm_response_guard_.pending()) +
      " publish_rate_hz=" + std::to_string(publish_rate_hz_) +
      " profile=" + profile_ +
      " live_env_complete=" + bool_text(
        live_environment_complete(environment) && control_source_ok(environment));
    if (exhibition_hold_active_ &&
      exhibition_dropout_started_ != SteadyClock::time_point{})
    {
      const double dropout_age = age(now, exhibition_dropout_started_);
      text += " vr_dropout_age_sec=" + std::to_string(dropout_age) +
        " reconnect_grace=" +
        std::string(dropout_age <= exhibition_reconnect_grace_sec_ ? "active" : "elapsed");
    }
    if (head_recenter_requested() && state_valid_) {
      text += " " + head_probe_diagnostics("active", latest_state_);
    }
    if (!head_probe_terminal_status_.empty()) {
      text += " last_probe_terminal={" + head_probe_terminal_status_ + "}";
    }
    publish_status(text, false);
  }

  void publish_status(const std::string & text, bool force_log)
  {
    const auto now = SteadyClock::now();
    if (text == last_status_text_ && !force_log &&
      last_status_time_ != SteadyClock::time_point{} && age(now, last_status_time_) < 1.0)
    {
      return;
    }
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
    const bool changed = text != last_status_text_;
    last_status_text_ = text;
    last_status_time_ = now;
    if (force_log || changed) {
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    }
  }

  bool send_commands_{false};
  bool enable_head_{false};
  bool enable_arms_{false};
  bool enable_locomotion_{false};
  bool velocity_status_127_probe_enabled_{false};
  bool enable_prepare_{false};
  bool prepare_enter_locomotion_{false};
  int prepare_speed_mode_{-1};
  bool commissioning_confirmed_{false};
  std::string commissioning_token_;
  std::string expected_vr_source_ip_;
  std::string vr_transport_;
  std::string transport_name_;
  std::string arm_sdk_frame_profile_{"legacy"};
  std::string locomotion_command_mode_{"loco_rpc"};
  std::string profile_;
  std::string network_interface_;
  bool require_prepare_{true};
  bool static_prepare_mode_{false};
  bool exhibition_session_mode_{false};
  bool sequential_control_enabled_{false};
  bool legs_quiet_{false};
  SequentialControl sequential_control_;
  ArmSdkPositions sequential_release_pose_{};
  SteadyClock::time_point sequential_zero_sent_{};
  bool exhibition_session_armed_on_start_{false};
  double exhibition_reconnect_grace_sec_{2.0};
  std::string head_topic_;
  std::string arm_topic_;
  std::string velocity_topic_;
  std::string joint_state_topic_;
  std::string motor_health_topic_;
  std::string deadman_topic_;
  std::string session_armed_topic_;
  std::string kill_topic_;
  std::string kill_request_topic_;
  std::string status_topic_;
  std::string debug_head_topic_;
  std::string debug_arm_topic_;
  std::string debug_velocity_topic_;
  double state_timeout_sec_{0.50};
  double motor_health_timeout_sec_{0.50};
  double command_timeout_sec_{0.25};
  double deadman_timeout_sec_{1.50};
  double kill_timeout_sec_{0.50};
  double publish_rate_hz_{100.0};
  double command_duration_sec_{1.0};
  double locomotion_rpc_rate_hz_{10.0};
  double wireless_controller_rate_hz_{20.0};
  VelocityLimits velocity_limits_;
  HeadLimits head_limits_;
  double arm_joint_delta_limit_rad_{0.25};
  double arm_joint_rate_rad_s_{1.0};
  bool arm_targets_absolute_{false};
  double arm_feedback_follow_command_delta_rad_{0.08};
  double arm_feedback_follow_min_progress_rad_{0.0015};
  double arm_feedback_follow_timeout_sec_{1.50};
  bool head_absolute_envelope_confirmed_{false};
  const HeadAbsoluteLimits head_absolute_limits_{};
  double max_head_tracking_seed_yaw_rad_{0.35};
  double max_head_tracking_seed_pitch_rad_{0.25};
  double max_head_seed_velocity_rad_s_{0.05};
  bool head_recenter_enabled_{false};
  bool head_recenter_confirmed_{false};
  double head_recenter_yaw_rate_rad_s_{0.08};
  double head_recenter_pitch_rate_rad_s_{0.05};
  double head_recenter_max_feedback_velocity_rad_s_{0.12};
  double head_recenter_seed_stability_rad_{0.01};
  double head_recenter_tolerance_rad_{0.02};
  double head_recenter_follow_tolerance_rad_{0.02};
  double head_recenter_follow_timeout_sec_{1.0};
  double head_recenter_timeout_sec_{35.0};
  bool head_ownership_probe_only_{true};
  bool head_ownership_probe_confirmed_{false};
  double head_probe_zero_weight_hold_sec_{0.35};
  double head_probe_weight_ramp_sec_{1.0};
  double head_probe_step_rad_{0.005};
  double head_probe_rate_rad_s_{0.005};
  double head_probe_max_feedback_velocity_rad_s_{0.04};
  double head_probe_min_progress_rad_{0.0015};
  double head_probe_seed_stability_rad_{0.003};
  double head_probe_other_joint_tolerance_rad_{0.003};
  double head_probe_ramp_other_joint_tolerance_rad_{0.015};
  double head_probe_claim_tolerance_rad_{0.003};
  double head_probe_direction_tolerance_rad_{0.0005};
  double head_probe_return_tolerance_rad_{0.0015};
  double head_probe_follow_timeout_sec_{1.5};
  double head_probe_total_timeout_sec_{8.0};
  int head_probe_required_feedback_samples_{3};
  int head_probe_seed_samples_{10};

  std::unique_ptr<RobotTransport> transport_;
  ArmSdkPositions latest_state_{};
  ArmSdkPositions held_head_target_{};
  ArmSdkPositions head_probe_full_weight_seed_{};
  bool head_probe_full_weight_seed_valid_{false};
  bool state_valid_{false};
  SteadyClock::time_point state_arrival_{};
  bool motor_health_seen_{false};
  bool motors_healthy_{false};
  SteadyClock::time_point motor_health_arrival_{};
  std::array<double, 2> latest_head_velocity_{};
  bool head_velocity_valid_{false};
  std::uint64_t state_sequence_{0};
  std::array<double, 2> latest_head_{};
  bool head_valid_{false};
  SteadyClock::time_point head_arrival_{};
  std::uint64_t head_sequence_{0};
  std::array<double, 10> latest_arm_{};
  std::array<double, 10> last_arm_command_{};
  ArmNeutralHold arm_neutral_hold_;
  ArmResponseGuard arm_response_guard_;
  bool arm_valid_{false};
  SteadyClock::time_point arm_arrival_{};
  std::uint64_t arm_sequence_{0};
  std::array<double, 3> latest_velocity_{};
  bool velocity_valid_{false};
  SteadyClock::time_point velocity_arrival_{};
  bool deadman_seen_{false};
  bool deadman_active_{false};
  SteadyClock::time_point deadman_arrival_{};
  bool session_control_seen_{false};
  bool session_control_armed_{false};
  bool exhibition_hold_active_{false};
  bool exhibition_arm_hold_sent_this_tick_{false};
  SteadyClock::time_point exhibition_dropout_started_{};
  bool kill_seen_{false};
  bool kill_signal_active_{true};
  bool local_kill_latched_{false};
  SteadyClock::time_point kill_arrival_{};
  bool prepared_{false};
  PrepareRearmGate prepare_rearm_gate_;
  bool prepare_in_progress_{false};
  std::atomic<bool> prepare_cancel_requested_{false};
  std::future<TransportResult> prepare_future_;
  bool head_claimed_{false};
  bool automatic_head_center_active_{false};
  bool automatic_head_center_complete_{false};
  bool head_tracking_seed_verified_{false};
  bool automatic_head_center_paused_{false};
  static constexpr std::size_t kHeadSeedStableSamples{5};
  static constexpr double kHeadSeedSettleTimeoutSec{2.0};
  SteadyClock::time_point head_seed_settle_started_{};
  std::uint64_t head_seed_settle_last_state_sequence_{0};
  std::size_t head_seed_settle_stable_samples_{0};
  bool locomotion_active_{false};
  SteadyClock::time_point velocity_127_probe_started_{};
  static constexpr double kCleanupRetryIntervalSec{0.5};
  SteadyClock::time_point last_locomotion_cleanup_attempt_{};
  SteadyClock::time_point last_safe_stop_status_time_{};
  std::array<double, 2> last_head_yaw_pitch_{};
  std::array<double, 2> head_origin_yaw_pitch_{};
  HeadRecenterState head_recenter_state_{HeadRecenterState::Disabled};
  ArmSdkPositions head_recenter_seed_{};
  ArmSdkPositions head_probe_seed_candidate_{};
  std::array<double, 2> head_recenter_target_yaw_pitch_{};
  std::array<double, 2> head_recenter_seed_candidate_yaw_pitch_{};
  std::uint64_t head_recenter_request_state_sequence_{0};
  std::uint64_t head_recenter_request_head_sequence_{0};
  std::uint64_t head_recenter_last_state_sequence_{0};
  std::uint64_t head_recenter_zero_state_sequence_{0};
  std::size_t head_recenter_stable_seed_samples_{0};
  std::size_t head_recenter_confirmation_samples_{0};
  SteadyClock::time_point head_recenter_request_time_{};
  SteadyClock::time_point head_recenter_started_time_{};
  SteadyClock::time_point head_recenter_last_advance_time_{};
  SteadyClock::time_point head_recenter_follow_wait_since_{};
  SteadyClock::time_point head_probe_phase_started_time_{};
  double head_probe_weight_{0.0};
  double head_probe_direction_{0.0};
  double head_probe_step_magnitude_rad_{0.0};
  std::size_t head_probe_feedback_samples_{0};
  double head_probe_last_feedback_progress_rad_{0.0};
  double head_probe_outbound_baseline_progress_rad_{0.0};
  std::string head_probe_terminal_status_;
  std::string last_safe_stop_detail_;
  std::array<double, 3> last_velocity_command_{};
  SteadyClock::time_point last_velocity_rpc_send_{};
  SteadyClock::time_point last_tick_{};
  SteadyClock::time_point last_status_time_{};
  std::string last_status_text_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kill_request_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    debug_head_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    debug_arm_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    debug_velocity_publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    head_subscription_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    arm_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    velocity_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_health_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr session_armed_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_subscription_;
  rclcpp::Service<Trigger>::SharedPtr prepare_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::Service<Trigger>::SharedPtr kill_service_;
  rclcpp::Service<Trigger>::SharedPtr recenter_head_service_;
  rclcpp::Service<Trigger>::SharedPtr probe_head_ownership_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_kill_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace r1_live_writer

int main(int argc, char ** argv)
{
  // In an acknowledged physical session, reject a wrong ROS middleware before
  // rclcpp creates a participant.  This prevents ROS CycloneDDS from binding
  // to Unitree's older libddsc before the in-node guards can run. Mock/dry-run
  // launches remain free to use the developer's default RMW.
  constexpr char kRequiredPreInitRmw[] = "rmw_fastrtps_cpp";
  const char * const physical_sdk_session = std::getenv("R1_PHYSICAL_SDK_SESSION");
  const char * const requested_rmw = std::getenv("RMW_IMPLEMENTATION");
  if (physical_sdk_session != nullptr && std::string(physical_sdk_session) == "1" &&
    (requested_rmw == nullptr || std::string(requested_rmw) != kRequiredPreInitRmw))
  {
    std::fprintf(
      stderr,
      "[BLOCKED] physical r1_live_writer requires RMW_IMPLEMENTATION=%s before rclcpp init\n",
      kRequiredPreInitRmw);
    return 2;
  }
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<r1_live_writer::R1LiveWriter>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("r1_live_writer"), "%s", exception.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
