// Copyright 2026
//
// Read-only Unitree R1 SDK boundary.
//
// This process deliberately does not include the R1 ArmSdk publisher or the
// locomotion client.  It subscribes to LowState (when the vendor SDK is
// available), republishes validated feedback to ROS 2, and evaluates arm
// trajectories in a dry-run safety gate.  A future writer must be a separate
// reviewed change and must not be enabled by merely changing a ROS parameter.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#if R1_SDK_TRANSPORT_HAS_SDK
#include <dds/dds.h>
#include <dlfcn.h>
#include <link.h>
#include <rmw/rmw.h>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#endif

namespace r1_sdk_transport
{

using Clock = std::chrono::steady_clock;

constexpr std::array<int, 26> kPhysicalSlots = {
  0, 1, 2, 3, 4, 5,
  6, 7, 8, 9, 10, 11,
  12, 13,
  15, 16, 17, 18, 19,
  22, 23, 24, 25, 26,
  29, 30};

constexpr std::array<int, 10> kArmSlots = {
  15, 16, 17, 18, 19, 22, 23, 24, 25, 26};

const std::array<const char *, 26> kDefaultJointNames = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
  "waist_roll_joint", "waist_yaw_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
  "head_pitch_joint", "head_yaw_joint"};

const std::array<const char *, 10> kDefaultArmNames = {
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"};

bool finite(const double value)
{
  return std::isfinite(value);
}

bool finite_vector(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), finite);
}

#if R1_SDK_TRANSPORT_HAS_SDK
constexpr char kRequiredRmwImplementation[] = "rmw_fastrtps_cpp";

struct LoadedDdsLibraries
{
  std::filesystem::path ddsc;
  std::filesystem::path ddscxx;
};

std::filesystem::path canonical_path_or_throw(
  const std::filesystem::path & path, const char * description)
{
  std::error_code error;
  const std::filesystem::path canonical = std::filesystem::canonical(path, error);
  if (error) {
    throw std::runtime_error(
            std::string("DDS ABI guard cannot resolve ") + description + " " +
            path.string() + ": " + error.message());
  }
  return canonical;
}

LoadedDdsLibraries loaded_dds_libraries_or_throw()
{
  Dl_info ddsc_info{};
  if (dladdr(reinterpret_cast<const void *>(&dds_create_participant), &ddsc_info) == 0 ||
    ddsc_info.dli_fname == nullptr || ddsc_info.dli_fname[0] == '\0')
  {
    throw std::runtime_error("DDS ABI guard cannot locate loaded libddsc.so.0");
  }
  const std::filesystem::path ddsc = canonical_path_or_throw(
    ddsc_info.dli_fname, "loaded libddsc.so.0 path");

  (void)dlerror();
  void * const ddscxx_handle = dlopen("libddscxx.so.0", RTLD_LAZY | RTLD_NOLOAD);
  if (ddscxx_handle == nullptr) {
    const char * const loader_error_text = dlerror();
    const std::string loader_error =
      loader_error_text == nullptr ? "unknown loader error" : loader_error_text;
    throw std::runtime_error(
            "DDS ABI guard cannot locate loaded libddscxx.so.0: " + loader_error);
  }

  link_map * ddscxx_link_map = nullptr;
  (void)dlerror();
  const int dlinfo_result = dlinfo(ddscxx_handle, RTLD_DI_LINKMAP, &ddscxx_link_map);
  std::string ddscxx_loaded_path;
  if (dlinfo_result == 0 && ddscxx_link_map != nullptr &&
    ddscxx_link_map->l_name != nullptr)
  {
    ddscxx_loaded_path = ddscxx_link_map->l_name;
  }
  const char * const dlinfo_error_text = dlinfo_result == 0 ? nullptr : dlerror();
  const std::string dlinfo_error =
    dlinfo_error_text == nullptr ? "" : dlinfo_error_text;
  dlclose(ddscxx_handle);

  if (dlinfo_result != 0 || ddscxx_loaded_path.empty()) {
    throw std::runtime_error(
            "DDS ABI guard cannot resolve loaded libddscxx.so.0 link map" +
            std::string(dlinfo_error.empty() ? "" : ": ") + dlinfo_error);
  }

  return {
    ddsc,
    canonical_path_or_throw(ddscxx_loaded_path, "loaded libddscxx.so.0 path")};
}

void verify_preinit_runtime_abi()
{
  const char * const configured_rmw = std::getenv("RMW_IMPLEMENTATION");
  if (configured_rmw == nullptr ||
    std::strcmp(configured_rmw, kRequiredRmwImplementation) != 0)
  {
    throw std::runtime_error(
            "DDS ABI guard requires explicit RMW_IMPLEMENTATION=" +
            std::string(kRequiredRmwImplementation) + "; got " +
            (configured_rmw == nullptr ? std::string("<unset>") :
            std::string(configured_rmw)));
  }

  const std::filesystem::path expected_directory = canonical_path_or_throw(
    R1_SDK_TRANSPORT_VENDOR_DDS_DIR, "compiled vendor DDS directory");
  const LoadedDdsLibraries loaded = loaded_dds_libraries_or_throw();
  if (loaded.ddsc.parent_path() != expected_directory ||
    loaded.ddscxx.parent_path() != expected_directory)
  {
    throw std::runtime_error(
            "DDS ABI guard rejected loaded libraries: expected directory=" +
            expected_directory.string() + " libddsc=" + loaded.ddsc.string() +
            " libddscxx=" + loaded.ddscxx.string());
  }
}

void verify_active_rmw_implementation()
{
  const char * const active_rmw = rmw_get_implementation_identifier();
  if (active_rmw == nullptr || std::strcmp(active_rmw, kRequiredRmwImplementation) != 0) {
    throw std::runtime_error(
            "DDS ABI guard requires active RMW " +
            std::string(kRequiredRmwImplementation) + "; got " +
            (active_rmw == nullptr ? std::string("<null>") : std::string(active_rmw)));
  }
}
#endif

class R1SdkTransport final : public rclcpp::Node
{
public:
  R1SdkTransport()
  : Node("r1_sdk_transport")
  {
    sdk_enabled_ = declare_parameter<bool>("sdk_enabled", false);
    dry_run_ = declare_parameter<bool>("dry_run", true);
    hardware_enabled_ = declare_parameter<bool>("hardware_enabled", false);
    commissioning_interlock_ = declare_parameter<bool>(
      "commissioning_interlock", false);
    arm_writer_enabled_ = declare_parameter<bool>("arm_writer_enabled", false);
    locomotion_writer_enabled_ = declare_parameter<bool>(
      "locomotion_writer_enabled", false);
    network_interface_ = declare_parameter<std::string>(
      "network_interface", "enxb4b024be59fe");
    lowstate_channel_ = declare_parameter<std::string>(
      "lowstate_channel", "rt/lf/lowstate");
    state_timeout_sec_ = declare_parameter<double>("state_timeout_sec", 0.5);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    active_timeout_sec_ = declare_parameter<double>("active_timeout_sec", 1.5);
    command_timeout_sec_ = declare_parameter<double>(
      "command_timeout_sec", 0.25);
    max_joint_step_rad_ = declare_parameter<double>(
      "max_joint_step_rad", 0.15);
    max_joint_velocity_rad_s_ = declare_parameter<double>(
      "max_joint_velocity_rad_s", 1.0);
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/r1/sdk/joint_states");
    motor_health_topic_ = declare_parameter<std::string>(
      "motor_health_topic", "/r1/sdk_transport/motors_healthy");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/r1/sdk_transport/status");
    arm_input_topic_ = declare_parameter<std::string>(
      "arm_input_topic", "/r1_kinematics_control/debug/arm_trajectory");
    active_topic_ = declare_parameter<std::string>(
      "active_topic", "/vr/teleop/active");
    debug_arm_topic_ = declare_parameter<std::string>(
      "debug_arm_topic", "/r1/sdk_transport/debug/arm_trajectory");
    physical_joint_names_ = declare_parameter<std::vector<std::string>>(
      "physical_joint_names", default_physical_joint_names());
    arm_joint_names_ = declare_parameter<std::vector<std::string>>(
      "arm_joint_names", default_arm_joint_names());
    arm_joint_min_rad_ = declare_parameter<std::vector<double>>(
      "arm_joint_min_rad", std::vector<double>(10, -3.141592653589793));
    arm_joint_max_rad_ = declare_parameter<std::vector<double>>(
      "arm_joint_max_rad", std::vector<double>(10, 3.141592653589793));

    validate_parameters();
    build_arm_index();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_, qos);
    motor_health_publisher_ = create_publisher<std_msgs::msg::Bool>(
      motor_health_topic_, qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, qos);
    debug_arm_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      debug_arm_topic_, qos);
    active_subscription_ = create_subscription<std_msgs::msg::Bool>(
      active_topic_, qos,
      std::bind(&R1SdkTransport::on_active, this, std::placeholders::_1));
    arm_subscription_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      arm_input_topic_, qos,
      std::bind(&R1SdkTransport::on_arm_command, this, std::placeholders::_1));

    if (sdk_enabled_) {
      initialize_sdk_read_only();
    }

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&R1SdkTransport::on_timer, this));

    publish_motor_health(false);
    publish_status(
      "startup read_only=true dry_run=" + bool_text(dry_run_) +
      " sdk_enabled=" + bool_text(sdk_enabled_) +
      " arm_writer=disabled locomotion_writer=disabled "
      "commissioning_interlock=" + bool_text(commissioning_interlock_) +
      " motorstate_nonzero=unknown motorstate=unavailable");
    RCLCPP_WARN(
      get_logger(),
      "R1 SDK transport is read-only: no rt/arm_sdk, rt/lowcmd, sport, or "
      "other command writer is created");
  }

  ~R1SdkTransport() override
  {
#if R1_SDK_TRANSPORT_HAS_SDK
    // Do not call ChannelFactory::Release() from a ROS process.  The SDK and
    // ROS 2 may share CycloneDDS symbols; tearing down the vendor singleton
    // while rclcpp is still unwinding can double-free DDS resources.  The
    // process is short-lived and the OS reclaims this reader safely at exit.
    // Deliberately release the ownership pointer so its destructor cannot
    // race the vendor callback thread during SIGINT; process exit reclaims
    // the reader.  This node is not a long-lived library component.
    sdk_subscriber_.release();
    (void)sdk_initialized_;
#endif
  }

private:
  static std::vector<std::string> default_physical_joint_names()
  {
    return std::vector<std::string>(
      kDefaultJointNames.begin(), kDefaultJointNames.end());
  }

  static std::vector<std::string> default_arm_joint_names()
  {
    return std::vector<std::string>(kDefaultArmNames.begin(), kDefaultArmNames.end());
  }

  static std::string bool_text(const bool value)
  {
    return value ? "true" : "false";
  }

  void validate_parameters()
  {
    // This is a hard interlock, not a soft warning.  The executable has no
    // writer implementation, so attempting to opt in must fail closed.
    if (!dry_run_ || hardware_enabled_ || commissioning_interlock_ ||
      arm_writer_enabled_ || locomotion_writer_enabled_)
    {
      throw std::runtime_error(
        "r1_sdk_transport is read-only; dry_run=true, hardware_enabled=false, "
        "commissioning_interlock=false, and both writer flags=false are required");
    }
    if (sdk_enabled_ && !R1_SDK_TRANSPORT_HAS_SDK) {
      throw std::runtime_error(
        "sdk_enabled=true but this build has no Unitree SDK2 headers/library");
    }
    if (network_interface_.empty() || lowstate_channel_.empty() ||
      joint_state_topic_.empty() || motor_health_topic_.empty() || status_topic_.empty())
    {
      throw std::runtime_error(
              "network_interface, lowstate_channel, and output topics are required");
    }
    if (!finite(state_timeout_sec_) || state_timeout_sec_ <= 0.0 ||
      !finite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
      !finite(active_timeout_sec_) || active_timeout_sec_ <= 0.0 ||
      !finite(command_timeout_sec_) || command_timeout_sec_ <= 0.0 ||
      !finite(max_joint_step_rad_) || max_joint_step_rad_ <= 0.0 ||
      !finite(max_joint_velocity_rad_s_) || max_joint_velocity_rad_s_ <= 0.0)
    {
      throw std::runtime_error("timeouts, rate, and safety limits must be finite and positive");
    }
    if (physical_joint_names_.size() != kPhysicalSlots.size()) {
      throw std::runtime_error("physical_joint_names must contain exactly 26 entries");
    }
    if (arm_joint_names_.size() != kArmSlots.size() ||
      arm_joint_min_rad_.size() != kArmSlots.size() ||
      arm_joint_max_rad_.size() != kArmSlots.size())
    {
      throw std::runtime_error(
        "arm_joint_names, arm_joint_min_rad, and arm_joint_max_rad must contain 10 entries");
    }
    if (!finite_vector(arm_joint_min_rad_) || !finite_vector(arm_joint_max_rad_)) {
      throw std::runtime_error("arm joint limits must be finite");
    }
    for (std::size_t index = 0; index < arm_joint_names_.size(); ++index) {
      if (arm_joint_names_[index].empty() ||
        !(arm_joint_min_rad_[index] < arm_joint_max_rad_[index]))
      {
        throw std::runtime_error("invalid arm joint name or lower/upper limit");
      }
    }
  }

  void build_arm_index()
  {
    for (std::size_t index = 0; index < arm_joint_names_.size(); ++index) {
      const auto result = arm_name_to_index_.emplace(arm_joint_names_[index], index);
      if (!result.second) {
        throw std::runtime_error("arm_joint_names contains duplicate names");
      }
    }
  }

  void initialize_sdk_read_only()
  {
#if R1_SDK_TRANSPORT_HAS_SDK
    // main() has already verified the explicit/active FastRTPS RMW and the
    // canonical paths of both vendor CycloneDDS libraries.  Keep this call
    // immediately before the sole Unitree ChannelFactory boundary.
    verify_preinit_runtime_abi();
    // Init only creates the SDK participant.  The only channel below is a
    // reader for LowState; no publisher/client is instantiated in this file.
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
    sdk_initialized_ = true;
    state_context_ = std::make_shared<StateContext>();
    sdk_subscriber_ = std::make_unique<
      unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      lowstate_channel_);
    const auto context = state_context_;
    sdk_subscriber_->InitChannel(
      [context](const void * raw) {
        R1SdkTransport::on_lowstate(context, raw);
      }, 10);
    RCLCPP_INFO(
      get_logger(), "subscribed read-only to %s on %s",
      lowstate_channel_.c_str(), network_interface_.c_str());
#else
    (void)lowstate_channel_;
#endif
  }

#if R1_SDK_TRANSPORT_HAS_SDK
  struct StateContext
  {
    mutable std::mutex mutex;
    unitree_hg::msg::dds_::LowState_ latest{};
    Clock::time_point arrival{};
    std::atomic<std::uint64_t> samples{0};
  };

  static void on_lowstate(
    const std::shared_ptr<StateContext> & context, const void * raw)
  {
    if (raw == nullptr || !context) {
      return;
    }
    const auto * state =
      static_cast<const unitree_hg::msg::dds_::LowState_ *>(raw);
    std::lock_guard<std::mutex> lock(context->mutex);
    context->latest = *state;
    context->arrival = Clock::now();
    ++context->samples;
  }
#endif

  bool copy_feedback(
    std::array<double, 26> & positions,
    std::array<double, 26> & velocities,
    std::array<std::uint32_t, 26> & motorstates) const
  {
#if R1_SDK_TRANSPORT_HAS_SDK
    if (!state_context_) {
      return false;
    }
    std::lock_guard<std::mutex> lock(state_context_->mutex);
    if (!have_state_locked()) {
      return false;
    }
    for (std::size_t index = 0; index < kPhysicalSlots.size(); ++index) {
      const auto & motor = state_context_->latest.motor_state().at(kPhysicalSlots[index]);
      positions[index] = static_cast<double>(motor.q());
      velocities[index] = static_cast<double>(motor.dq());
      motorstates[index] = motor.motorstate();
      if (!finite(positions[index]) || !finite(velocities[index])) {
        return false;
      }
    }
    return true;
#else
    (void)positions;
    (void)velocities;
    (void)motorstates;
    return false;
#endif
  }

  bool have_state_locked() const
  {
#if R1_SDK_TRANSPORT_HAS_SDK
    if (!sdk_enabled_ || !state_context_ || state_context_->samples == 0 ||
      state_context_->arrival == Clock::time_point{})
    {
      return false;
    }
    const double age = std::chrono::duration<double>(
      Clock::now() - state_context_->arrival).count();
    return age <= state_timeout_sec_;
#else
    return false;
#endif
  }

  bool copy_raw_mode_state(std::uint8_t & mode_pr, std::uint8_t & mode_machine) const
  {
#if R1_SDK_TRANSPORT_HAS_SDK
    if (!state_context_) {
      return false;
    }
    std::lock_guard<std::mutex> lock(state_context_->mutex);
    if (!have_state_locked()) {
      return false;
    }
    // These are intentionally reported as raw firmware values.  The bundled
    // R1 SDK does not document a complete mode/state transition table, so the
    // read-only transport must not reinterpret either value as "ready" or
    // use it to authorize a command writer.
    mode_pr = state_context_->latest.mode_pr();
    mode_machine = state_context_->latest.mode_machine();
    return true;
#else
    (void)mode_pr;
    (void)mode_machine;
    return false;
#endif
  }

  void on_active(const std_msgs::msg::Bool::SharedPtr message)
  {
    const bool active = message && message->data;
    const auto now = Clock::now();
    const bool transitioned = !active_seen_ || active != active_;
    active_ = active;
    active_seen_ = true;
    active_arrival_ = now;
    if (!active) {
      last_arm_command_.clear();
      last_arm_command_time_ = Clock::time_point{};
      if (transitioned) {
        publish_status("deadman=false arm_gate_reset");
      }
    } else if (transitioned) {
      publish_status("deadman=true; waiting for fresh LowState seed");
    }
  }

  void on_arm_command(const trajectory_msgs::msg::JointTrajectory::SharedPtr message)
  {
    if (!message || message->points.empty()) {
      publish_status("arm_rejected=empty_trajectory");
      return;
    }
    if (!active_is_fresh()) {
      publish_status("arm_rejected=deadman_inactive_or_stale");
      return;
    }
    std::array<double, 26> feedback{};
    std::array<double, 26> velocities{};
    std::array<std::uint32_t, 26> motorstates{};
    if (!copy_feedback(feedback, velocities, motorstates)) {
      publish_status("arm_rejected=lowstate_missing_stale_or_nonfinite");
      return;
    }

    const auto & point = message->points.front();
    if (message->joint_names.size() != point.positions.size() ||
      message->joint_names.empty())
    {
      publish_status("arm_rejected=joint_name_count");
      return;
    }
    std::vector<double> target(point.positions.begin(), point.positions.end());
    if (!finite_vector(target)) {
      publish_status("arm_rejected=nonfinite_joint_position");
      return;
    }
    std::vector<std::size_t> configured_indices;
    configured_indices.reserve(message->joint_names.size());
    for (const auto & name : message->joint_names) {
      const auto found = arm_name_to_index_.find(name);
      if (found == arm_name_to_index_.end()) {
        publish_status("arm_rejected=unknown_arm_joint:" + name);
        return;
      }
      if (std::find(configured_indices.begin(), configured_indices.end(), found->second) !=
        configured_indices.end())
      {
        publish_status("arm_rejected=duplicate_joint_name");
        return;
      }
      configured_indices.push_back(found->second);
    }

    const auto now = Clock::now();
    double dt = 0.01;
    if (last_arm_command_time_ != Clock::time_point{}) {
      dt = std::chrono::duration<double>(now - last_arm_command_time_).count();
      dt = std::max(0.001, std::min(0.2, dt));
    }
    const double max_step = std::min(
      max_joint_step_rad_, max_joint_velocity_rad_s_ * dt);
    std::vector<double> previous = last_arm_command_;
    if (previous.size() != arm_joint_names_.size()) {
      previous.resize(arm_joint_names_.size());
      for (std::size_t index = 0; index < previous.size(); ++index) {
        previous[index] = feedback[arm_feedback_index(index)];
      }
    }

    bool clamped = false;
    for (std::size_t item = 0; item < target.size(); ++item) {
      const std::size_t index = configured_indices[item];
      const double bounded = std::max(
        arm_joint_min_rad_[index], std::min(arm_joint_max_rad_[index], target[item]));
      if (bounded != target[item]) {
        clamped = true;
      }
      target[item] = bounded;
      const double delta = target[item] - previous[index];
      const double limited_delta = std::max(-max_step, std::min(max_step, delta));
      if (limited_delta != delta) {
        clamped = true;
      }
      previous[index] += limited_delta;
      target[item] = previous[index];
    }
    last_arm_command_ = previous;
    last_arm_command_time_ = now;

    trajectory_msgs::msg::JointTrajectory debug;
    debug.header.stamp = get_clock()->now();
    debug.joint_names = message->joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint debug_point;
    debug_point.positions = target;
    debug_point.time_from_start = point.time_from_start;
    debug.points.push_back(debug_point);
    debug_arm_publisher_->publish(debug);
    publish_status(
      std::string("arm_debug=accepted seed=lowstate limits=") +
      (clamped ? "clamped" : "within_limits"));
  }

  std::size_t arm_feedback_index(const std::size_t arm_index) const
  {
    // R1 lowstate array order is explicit and differs from the 26-joint ROS
    // order only at the reserved IDL slots.  The physical feedback array is
    // ordered as kPhysicalSlots above; convert the command slot to that index.
    const int slot = kArmSlots.at(arm_index);
    const auto found = std::find(kPhysicalSlots.begin(), kPhysicalSlots.end(), slot);
    if (found == kPhysicalSlots.end()) {
      throw std::logic_error("R1 arm slot is not present in physical feedback map");
    }
    return static_cast<std::size_t>(std::distance(kPhysicalSlots.begin(), found));
  }

  bool active_is_fresh() const
  {
    if (!active_seen_ || !active_) {
      return false;
    }
    const double age = std::chrono::duration<double>(Clock::now() - active_arrival_).count();
    return age <= active_timeout_sec_;
  }

  std::uint64_t state_sample_count() const
  {
#if R1_SDK_TRANSPORT_HAS_SDK
    return state_context_ ? state_context_->samples.load() : 0;
#else
    return 0;
#endif
  }

  void on_timer()
  {
    bool feedback_ready = false;
    bool motors_healthy = false;
    if (sdk_enabled_) {
      std::array<double, 26> positions{};
      std::array<double, 26> velocities{};
      std::array<std::uint32_t, 26> motorstates{};
      feedback_ready = copy_feedback(positions, velocities, motorstates);
      if (!feedback_ready) {
        publish_status(
          "lowstate=missing_stale_or_nonfinite motorstate_nonzero=unknown "
          "motorstate=unavailable");
      } else {
        sensor_msgs::msg::JointState message;
        message.header.stamp = get_clock()->now();
        message.name = physical_joint_names_;
        message.position.assign(positions.begin(), positions.end());
        message.velocity.assign(velocities.begin(), velocities.end());
        joint_state_publisher_->publish(message);
        std::uint8_t mode_pr = 0;
        std::uint8_t mode_machine = 0;
        const bool have_modes = copy_raw_mode_state(mode_pr, mode_machine);
        std::size_t motorstate_nonzero = 0;
        std::string motorstate_detail;
        for (std::size_t index = 0; index < motorstates.size(); ++index) {
          if (motorstates[index] == 0U) {
            continue;
          }
          if (!motorstate_detail.empty()) {
            motorstate_detail += ',';
          }
          motorstate_detail += std::to_string(kPhysicalSlots[index]) + ':' +
            std::to_string(motorstates[index]);
          ++motorstate_nonzero;
        }
        motors_healthy = motorstate_nonzero == 0;
        std::string status =
          "lowstate=ok samples=" + std::to_string(state_sample_count()) +
          " motorstate_nonzero=" + std::to_string(motorstate_nonzero) +
          " motorstate=" +
          (motorstate_detail.empty() ? std::string("none") : motorstate_detail);
        if (have_modes) {
          status += " mode_pr_raw=" + std::to_string(
            static_cast<unsigned int>(mode_pr));
          status += " mode_machine_raw=" + std::to_string(
            static_cast<unsigned int>(mode_machine));
        } else {
          status += " mode_raw=unavailable";
        }
        publish_status(status);
      }
    } else {
      publish_status(
        "lowstate=disabled sdk_enabled=false motorstate_nonzero=unknown "
        "motorstate=unavailable");
    }
    // Publish on every timer tick. Consumers must independently enforce a
    // bounded arrival age, so a stopped reader can never leave a stale true
    // value authorizing physical output.
    publish_motor_health(feedback_ready && motors_healthy);

    // A stale/invalid physical feedback stream is itself a stop condition;
    // do not leave the arm gate armed while the seed source is unavailable.
    if (active_ && sdk_enabled_ && !feedback_ready) {
      active_ = false;
      last_arm_command_.clear();
      last_arm_command_time_ = Clock::time_point{};
      publish_status("watchdog=arm_gate_reset_lowstate_stale");
    }

    if (active_seen_ && (!active_is_fresh() ||
      (last_arm_command_time_ != Clock::time_point{} &&
      std::chrono::duration<double>(Clock::now() - last_arm_command_time_).count() >
      command_timeout_sec_)))
    {
      if (active_) {
        active_ = false;
        last_arm_command_.clear();
        last_arm_command_time_ = Clock::time_point{};
        publish_status("watchdog=arm_gate_reset_no_fresh_deadman_or_command");
      }
    }
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
  }

  void publish_motor_health(const bool healthy)
  {
    std_msgs::msg::Bool message;
    message.data = healthy;
    motor_health_publisher_->publish(message);
  }

  bool sdk_enabled_{false};
  bool dry_run_{true};
  bool hardware_enabled_{false};
  bool commissioning_interlock_{false};
  bool arm_writer_enabled_{false};
  bool locomotion_writer_enabled_{false};
  std::string network_interface_;
  std::string lowstate_channel_;
  double state_timeout_sec_{0.5};
  double publish_rate_hz_{20.0};
  double active_timeout_sec_{1.5};
  double command_timeout_sec_{0.25};
  double max_joint_step_rad_{0.15};
  double max_joint_velocity_rad_s_{1.0};
  std::string joint_state_topic_;
  std::string motor_health_topic_;
  std::string status_topic_;
  std::string arm_input_topic_;
  std::string active_topic_;
  std::string debug_arm_topic_;
  std::vector<std::string> physical_joint_names_;
  std::vector<std::string> arm_joint_names_;
  std::vector<double> arm_joint_min_rad_;
  std::vector<double> arm_joint_max_rad_;
  std::map<std::string, std::size_t> arm_name_to_index_;

  bool active_seen_{false};
  bool active_{false};
  Clock::time_point active_arrival_{};
  std::vector<double> last_arm_command_;
  Clock::time_point last_arm_command_time_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_health_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr debug_arm_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_subscription_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

#if R1_SDK_TRANSPORT_HAS_SDK
  std::shared_ptr<StateContext> state_context_;
  bool sdk_initialized_{false};
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>
    sdk_subscriber_;
#endif
};

}  // namespace r1_sdk_transport

int main(int argc, char ** argv)
{
  const bool abi_check_only =
    argc == 2 && std::strcmp(argv[1], "--abi-check-only") == 0;
  const bool rmw_check_only =
    argc == 2 && std::strcmp(argv[1], "--rmw-check-only") == 0;
#if R1_SDK_TRANSPORT_HAS_SDK
  try {
    r1_sdk_transport::verify_preinit_runtime_abi();
    // Resolving the rmw_implementation proxy only loads and identifies the
    // selected plugin; it does not create a ROS context or network endpoint.
    r1_sdk_transport::verify_active_rmw_implementation();
    if (abi_check_only) {
      std::printf("DDS ABI guard accepted explicit FastRTPS and vendor libraries\n");
      return 0;
    }
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "r1_sdk_transport pre-init failure: %s\n", exception.what());
    return 1;
  }
#else
  if (abi_check_only || rmw_check_only) {
    std::printf("DDS ABI guard not applicable: Unitree SDK was not compiled\n");
    return 0;
  }
#endif

  bool ros_initialized = false;
  try {
    rclcpp::init(argc, argv);
    ros_initialized = true;
#if R1_SDK_TRANSPORT_HAS_SDK
    r1_sdk_transport::verify_active_rmw_implementation();
#endif
    if (rmw_check_only) {
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
      std::printf("DDS ABI guard accepted active FastRTPS after ROS initialization\n");
      return 0;
    }
    auto node = std::make_shared<r1_sdk_transport::R1SdkTransport>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "r1_sdk_transport runtime failure: %s\n", exception.what());
    if (ros_initialized && rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  } catch (...) {
    std::fprintf(stderr, "r1_sdk_transport runtime failure: unknown exception\n");
    if (ros_initialized && rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (ros_initialized && rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
