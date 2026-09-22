#include "r1_live_writer/transport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <limits>
#include <sstream>
#include <thread>

#if R1_LIVE_WRITER_HAS_SDK
#include <dds/dds.h>
#include <dlfcn.h>
#include <link.h>
#include <unitree/dds_wrapper/robots/r1/r1.h>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/r1/loco/r1_loco_client.hpp>
#endif

namespace r1_live_writer
{

double detail::descending_arm_release_weight(
  double start_weight, std::size_t completed_steps, std::size_t total_steps)
{
  if (!std::isfinite(start_weight) || start_weight < 0.0 || start_weight > 1.0 ||
    total_steps == 0 || completed_steps > total_steps)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return start_weight *
         (1.0 - static_cast<double>(completed_steps) / static_cast<double>(total_steps));
}

namespace
{

bool finite_head(const ArmSdkPositions & positions)
{
  return std::all_of(positions.begin(), positions.end(), [](double value) {
    return std::isfinite(value);
  });
}

bool valid_arm_weight(double weight)
{
  return std::isfinite(weight) && weight >= 0.0 && weight <= 1.0;
}

TransportResult success(const std::string & detail)
{
  return {true, detail};
}

TransportResult failure(const std::string & detail)
{
  return {false, detail};
}

#if R1_LIVE_WRITER_HAS_SDK
// Immutable allow-list for the SDK selected by this target. Runtime loader
// settings must not be able to redefine which CycloneDDS build is trusted.
// Keep these paths in lockstep with UNITREE_SDK_LIB_DIR in CMakeLists.txt.
#if defined(__x86_64__)
constexpr char kCompiledVendorDdsDirectory[] =
  "/home/unitree/Unitree_Project/Legacy_Robotics/robotics/unitree_sdk/"
  "unitree_sdk2/thirdparty/lib/x86_64";
#elif defined(__aarch64__)
constexpr char kCompiledVendorDdsDirectory[] =
  "/home/unitree/Unitree_Project/Legacy_Robotics/robotics/unitree_sdk/"
  "unitree_sdk2/thirdparty/lib/aarch64";
#else
constexpr char kCompiledVendorDdsDirectory[] = "";
#endif

TransportResult verify_vendor_dds_libraries()
{
  namespace fs = std::filesystem;

  if (kCompiledVendorDdsDirectory[0] == '\0') {
    return failure("Unitree DDS runtime guard has no vendor directory for this architecture");
  }

  std::error_code path_error;
  const fs::path expected_directory = fs::canonical(kCompiledVendorDdsDirectory, path_error);
  if (path_error) {
    return failure(
      "Unitree DDS runtime guard cannot resolve compiled vendor directory " +
      std::string(kCompiledVendorDdsDirectory) + ": " + path_error.message());
  }

  Dl_info ddsc_info{};
  if (dladdr(reinterpret_cast<const void *>(&dds_create_participant), &ddsc_info) == 0 ||
    ddsc_info.dli_fname == nullptr || ddsc_info.dli_fname[0] == '\0')
  {
    return failure("Unitree DDS runtime guard cannot locate loaded libddsc.so.0");
  }
  const fs::path ddsc_path = fs::canonical(ddsc_info.dli_fname, path_error);
  if (path_error) {
    return failure(
      "Unitree DDS runtime guard cannot resolve loaded libddsc.so.0 path " +
      std::string(ddsc_info.dli_fname) + ": " + path_error.message());
  }

  void * const ddscxx_handle = dlopen("libddscxx.so.0", RTLD_LAZY | RTLD_NOLOAD);
  if (ddscxx_handle == nullptr) {
    const char * const loader_error = dlerror();
    return failure(
      "Unitree DDS runtime guard cannot locate loaded libddscxx.so.0" +
      std::string(loader_error == nullptr ? "" : ": ") +
      std::string(loader_error == nullptr ? "" : loader_error));
  }

  link_map * ddscxx_link_map = nullptr;
  (void)dlerror();
  const int dlinfo_result = dlinfo(ddscxx_handle, RTLD_DI_LINKMAP, &ddscxx_link_map);
  std::string ddscxx_loaded_path;
  if (dlinfo_result == 0 && ddscxx_link_map != nullptr && ddscxx_link_map->l_name != nullptr) {
    ddscxx_loaded_path = ddscxx_link_map->l_name;
  }
  const char * const dlinfo_error_text = dlinfo_result == 0 ? nullptr : dlerror();
  const std::string dlinfo_error = dlinfo_error_text == nullptr ? "" : dlinfo_error_text;
  dlclose(ddscxx_handle);
  if (dlinfo_result != 0 || ddscxx_loaded_path.empty()) {
    return failure(
      "Unitree DDS runtime guard cannot resolve loaded libddscxx.so.0 link map" +
      std::string(dlinfo_error.empty() ? "" : ": ") + dlinfo_error);
  }

  const fs::path ddscxx_path = fs::canonical(ddscxx_loaded_path, path_error);
  if (path_error) {
    return failure(
      "Unitree DDS runtime guard cannot resolve loaded libddscxx.so.0 path " +
      ddscxx_loaded_path + ": " + path_error.message());
  }

  if (ddsc_path.parent_path() != expected_directory ||
    ddscxx_path.parent_path() != expected_directory)
  {
    return failure(
      "Unitree DDS runtime guard rejected libraries: expected directory=" +
      expected_directory.string() + " libddsc=" + ddsc_path.string() +
      " libddscxx=" + ddscxx_path.string());
  }

  return success(
    "Unitree DDS runtime guard accepted libddsc and libddscxx from " +
    expected_directory.string());
}
#endif

}  // namespace

TransportResult MockTransport::initialize(
  const std::string & network_interface, bool head, bool locomotion)
{
  if (network_interface.empty()) {
    return failure("mock initialize rejected empty network interface");
  }
  initialized_ = true;
  head_enabled_ = head;
  locomotion_enabled_ = locomotion;
  return success("mock initialized");
}

TransportResult MockTransport::prepare(
  const CancelCheck & should_cancel, bool enter_locomotion)
{
  (void)enter_locomotion;
  if (!initialized_ || !locomotion_enabled_) {
    return failure("mock locomotion transport is not initialized");
  }
  if (should_cancel && should_cancel()) {
    return failure("mock prepare cancelled");
  }
  ++prepare_calls_;
  return success("mock StandUp recorded");
}

TransportResult MockTransport::set_velocity(
  double forward, double lateral, double yaw, double duration_sec)
{
  if (!initialized_ || !locomotion_enabled_) {
    return failure("mock locomotion transport is not initialized");
  }
  if (!std::isfinite(forward) || !std::isfinite(lateral) || !std::isfinite(yaw) ||
    !std::isfinite(duration_sec) || duration_sec <= 0.0)
  {
    return failure("mock rejected invalid velocity");
  }
  last_velocity_ = {forward, lateral, yaw};
  ++velocity_calls_;
  return success("mock SetVelocity recorded");
}

TransportResult MockTransport::stop_locomotion()
{
  if (!initialized_ || !locomotion_enabled_) {
    return failure("mock locomotion transport is not initialized");
  }
  last_velocity_ = {0.0, 0.0, 0.0};
  ++stop_calls_;
  return success("mock StopMove recorded");
}

TransportResult MockTransport::seed_head(const ArmSdkPositions & positions)
{
  return seed_head_weighted(positions, 1.0);
}

TransportResult MockTransport::seed_head_weighted(
  const ArmSdkPositions & positions, double weight)
{
  if (!initialized_ || !head_enabled_) {
    return failure("mock head transport is not initialized");
  }
  if (!finite_head(positions)) {
    return failure("mock rejected nonfinite head seed");
  }
  if (!valid_arm_weight(weight)) {
    return failure("mock rejected head seed weight outside finite [0,1]");
  }
  last_head_ = positions;
  last_head_weight_ = weight;
  head_seeded_ = true;
  ++head_seed_calls_;
  return success("mock weighted ArmSdk seed recorded");
}

TransportResult MockTransport::command_head(const ArmSdkPositions & positions)
{
  return command_head_weighted(positions, 1.0);
}

TransportResult MockTransport::command_head_weighted(
  const ArmSdkPositions & positions, double weight)
{
  if (!head_seeded_) {
    return failure("mock head command rejected before seed");
  }
  if (!finite_head(positions)) {
    return failure("mock rejected nonfinite head command");
  }
  if (!valid_arm_weight(weight)) {
    return failure("mock rejected head command weight outside finite [0,1]");
  }
  last_head_ = positions;
  last_head_weight_ = weight;
  ++head_command_calls_;
  return success("mock weighted ArmSdk command recorded");
}

TransportResult MockTransport::hold_head(const ArmSdkPositions & positions)
{
  if (!head_seeded_) {
    return failure("mock head hold rejected before seed");
  }
  if (!finite_head(positions)) {
    return failure("mock rejected nonfinite head hold");
  }
  last_head_ = positions;
  ++head_hold_calls_;
  return success("mock ArmSdk hold recorded at current weight");
}

TransportResult MockTransport::release_head(const ArmSdkPositions & positions)
{
  if (!head_seeded_) {
    return success("mock ArmSdk release skipped; head was never seeded");
  }
  if (!finite_head(positions)) {
    return failure("mock rejected nonfinite head release");
  }
  last_head_ = positions;
  last_head_weight_ = 0.0;
  head_seeded_ = false;
  ++head_release_calls_;
  return success("mock ArmSdk weight release recorded");
}

class SdkTransport::Impl
{
public:
  bool initialized{false};
  bool head_enabled{false};
  bool locomotion_enabled{false};
  bool head_seeded{false};
  // Set before any positive-weight ArmSdk publisher boundary. An exception
  // or timeout cannot prove that the frame was not queued, so only a confirmed
  // weight-zero publication may clear this cleanup debt.
  bool head_maybe_active{false};
  // The most recently accepted/requested weight bounds a normal release ramp.
  // A failed positive publication is treated as ambiguous and is relinquished
  // directly at weight zero instead of risking a ramp from an invented state.
  float head_release_weight{0.0F};
  bool head_weight_delivery_ambiguous{false};
  // StandUp and Start are asynchronous high-level locomotion commands. Until
  // the requested mode is confirmed, a timeout/cancellation/nonzero reply is
  // ambiguous and a post-worker StopMove is owed just as it is after an
  // ambiguous SetVelocity RPC.
  bool prepare_maybe_active{false};
  // Set before entering SetVelocity().  A timeout, exception, or nonzero RPC
  // result cannot prove that the robot rejected the request, so StopMove is
  // still owed until it succeeds.
  bool velocity_maybe_active{false};
  ArmSdkPositions last_head{};

#if R1_LIVE_WRITER_HAS_SDK
  std::unique_ptr<unitree::robot::r1::publisher::ArmSdk> arm_sdk;
  std::unique_ptr<unitree::robot::r1::LocoClient> loco;

  TransportResult publish_arm(const ArmSdkPositions & positions, float weight)
  {
    if (!arm_sdk || !finite_head(positions) || !valid_arm_weight(weight)) {
      return failure("ArmSdk unavailable, positions nonfinite, or weight outside finite [0,1]");
    }

    // These are exactly the values and the 13-joint order in Unitree's R1
    // r1_arm_sdk_dds_example.cpp.  They are installed only at the first
    // explicitly authorized seed; construction never activates ArmSdk.
    static constexpr std::array<float, 13> kVendorKp = {
      50.0F, 50.0F, 40.0F, 40.0F, 30.0F,
      50.0F, 50.0F, 40.0F, 40.0F, 30.0F,
      50.0F, 15.0F, 15.0F};
    static constexpr std::array<float, 13> kVendorKd = {
      2.0F, 2.0F, 2.0F, 2.0F, 2.0F,
      2.0F, 2.0F, 2.0F, 2.0F, 2.0F,
      3.0F, 1.0F, 1.0F};

    bool lock_held = false;
    try {
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(50);
      while (!arm_sdk->trylock()) {
        if (std::chrono::steady_clock::now() >= deadline) {
          return failure("ArmSdk realtime publisher lock timeout");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      lock_held = true;
      arm_sdk->weight(weight);
      for (std::size_t index = 0; index < arm_sdk->JOINTS.size(); ++index) {
        const int joint = static_cast<int>(arm_sdk->JOINTS[index]);
        auto & command = arm_sdk->msg_.motor_cmd().at(joint);
        command.q(static_cast<float>(positions[index]));
        command.kp(kVendorKp[index]);
        command.kd(kVendorKd[index]);
        command.dq(0.0F);
        command.tau(0.0F);
      }
      arm_sdk->unlockAndPublish();
      lock_held = false;
      last_head = positions;
      return success("ArmSdk frame queued");
    } catch (const std::exception & exception) {
      if (lock_held) {
        try {
          arm_sdk->unlock();
        } catch (...) {
          // Preserve the original failure. A later release/destructor retry is
          // still bounded and the physical E-stop remains authoritative.
        }
      }
      return failure(std::string("ArmSdk frame exception: ") + exception.what());
    } catch (...) {
      if (lock_held) {
        try {
          arm_sdk->unlock();
        } catch (...) {
        }
      }
      return failure("ArmSdk frame unknown exception");
    }
  }
#endif
};

SdkTransport::SdkTransport()
: impl_(std::make_unique<Impl>())
{
  // Deliberately empty: no Unitree singleton, client, channel, or publisher.
}

SdkTransport::~SdkTransport()
{
#if R1_LIVE_WRITER_HAS_SDK
  // Retry the official stop once during teardown when StandUp, Start, or
  // SetVelocity may have reached the robot. Destruction must remain noexcept even if the
  // SDK reports a transport exception.
  if (impl_ && (impl_->prepare_maybe_active || impl_->velocity_maybe_active) && impl_->loco) {
    try {
      (void)stop_locomotion();
    } catch (...) {
      // The physical E-stop remains the final authority.  There is no safe
      // additional SDK action available from a destructor after this retry.
    }
  }
  // A normal ROS shutdown must not leave this process's last full-weight
  // ArmSdk frame as its final ownership request.  This uses only the last
  // already-commanded hold target; it never invents a new pose.
  if (impl_ && (impl_->head_seeded || impl_->head_maybe_active) && impl_->arm_sdk) {
    try {
      (void)release_head(impl_->last_head);
    } catch (...) {
      // Never allow an SDK exception to escape a destructor.
    }
  }
#endif
}

TransportResult SdkTransport::initialize(
  const std::string & network_interface, bool head, bool locomotion)
{
  if (impl_->initialized) {
    if ((head && !impl_->head_enabled) || (locomotion && !impl_->locomotion_enabled)) {
      return failure("SDK transport cannot add a feature after initialization");
    }
    return success("SDK transport already initialized");
  }
  if (network_interface.empty()) {
    return failure("SDK initialize rejected empty network interface");
  }
#if R1_LIVE_WRITER_HAS_SDK
  try {
    const TransportResult dds_guard = verify_vendor_dds_libraries();
    if (!dds_guard.ok) {
      return dds_guard;
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    if (locomotion) {
      impl_->loco = std::make_unique<unitree::robot::r1::LocoClient>();
      impl_->loco->Init();
      impl_->loco->SetTimeout(0.5F);
    }
    if (head) {
      impl_->arm_sdk =
        std::make_unique<unitree::robot::r1::publisher::ArmSdk>("rt/arm_sdk");
    }
    impl_->head_enabled = head;
    impl_->locomotion_enabled = locomotion;
    impl_->initialized = true;
    return success("Unitree SDK transport initialized without sending a command");
  } catch (const std::exception & exception) {
    return failure(std::string("Unitree SDK initialization failed: ") + exception.what());
  } catch (...) {
    return failure("Unitree SDK initialization failed with unknown exception");
  }
#else
  (void)head;
  (void)locomotion;
  return failure("SDK transport was not compiled; Unitree SDK2 unavailable");
#endif
}

TransportResult SdkTransport::prepare(
  const CancelCheck & should_cancel, bool enter_locomotion)
{
#if R1_LIVE_WRITER_HAS_SDK
  if (!impl_->initialized || !impl_->locomotion_enabled || !impl_->loco) {
    return failure("R1 LocoClient is not initialized");
  }
  try {
    if (should_cancel && should_cancel()) {
      return failure("R1 prepare cancelled before StandUp");
    }
    // Mark the high-level transition potentially active before crossing the
    // RPC boundary. Only a successful StopMove may clear it; every other
    // result is ambiguous and must remain fail-closed.
    impl_->prepare_maybe_active = true;
    const int32_t result = impl_->loco->StandUp();
    if (result != 0) {
      return failure("R1 StandUp error=" + std::to_string(result));
    }

    // StandUp acknowledgement only confirms the RPC. Require the reported
    // high-level state to settle at the R1 standing FSM for several
    // consecutive samples before entering locomotion mode.
    constexpr int kRequiredStableFsmSamples = 5;
    constexpr int kStandingFsmId = 4;
    constexpr int kLocomotionFsmId = 811;
    constexpr auto kFsmPollPeriod = std::chrono::milliseconds(100);
    constexpr auto kFsmDeadline = std::chrono::seconds(6);

    const auto wait_for_stable_fsm = [&](int expected_fsm_id, const char * label) {
      const auto deadline = std::chrono::steady_clock::now() + kFsmDeadline;
      int stable_fsm_samples = 0;
      int last_fsm_id = -1;
      int32_t last_get_fsm_result = 0;
      while (std::chrono::steady_clock::now() < deadline) {
        if (should_cancel && should_cancel()) {
          if (std::string(label) == "StandUp") {
            return failure("R1 StandUp FSM confirmation cancelled");
          }
          return failure("R1 Start FSM confirmation cancelled");
        }
        int fsm_id = -1;
        last_get_fsm_result = impl_->loco->GetFsmId(fsm_id);
        if (should_cancel && should_cancel()) {
          if (std::string(label) == "StandUp") {
            return failure("R1 StandUp FSM confirmation cancelled");
          }
          return failure("R1 Start FSM confirmation cancelled");
        }
        if (last_get_fsm_result == 0) {
          last_fsm_id = fsm_id;
        }
        const bool expected_state =
          (expected_fsm_id == kStandingFsmId && fsm_id == kStandingFsmId) ||
          (expected_fsm_id == kLocomotionFsmId && fsm_id == kLocomotionFsmId);
        if (last_get_fsm_result == 0 && expected_state) {
          ++stable_fsm_samples;
          if (stable_fsm_samples >= kRequiredStableFsmSamples) {
            return TransportResult{true, std::string("R1 ") + label +
              " confirmed by stable FSM " + std::to_string(expected_fsm_id) +
              " samples=" + std::to_string(stable_fsm_samples)};
          }
        } else {
          stable_fsm_samples = 0;
        }
        const auto wake = std::min(
          deadline, std::chrono::steady_clock::now() + kFsmPollPeriod);
        std::this_thread::sleep_until(wake);
      }
      return TransportResult{false, std::string("R1 ") + label +
        " FSM confirmation timeout last_get_result=" +
        std::to_string(last_get_fsm_result) + " last_fsm_id=" +
        std::to_string(last_fsm_id) + " stable_samples=" +
        std::to_string(stable_fsm_samples)};
    };

    const TransportResult stand_up_state = wait_for_stable_fsm(kStandingFsmId, "StandUp");
    if (!stand_up_state.ok) {
      return stand_up_state;
    }

    if (!enter_locomotion) {
      // Head/arms-only commissioning needs the stable standing pose but must
      // not switch the legs into sport mode or create a StopMove cleanup debt.
      impl_->prepare_maybe_active = false;
      return stand_up_state;
    }

    if (should_cancel && should_cancel()) {
      return failure("R1 prepare cancelled before Start");
    }
    const int32_t start_result = impl_->loco->Start();
    if (start_result != 0) {
      return failure("R1 Start error=" + std::to_string(start_result));
    }

    const TransportResult locomotion_state =
      wait_for_stable_fsm(kLocomotionFsmId, "Start");
    if (!locomotion_state.ok) {
      return locomotion_state;
    }

    return success(
      stand_up_state.detail + "; " + locomotion_state.detail);
  } catch (const std::exception & exception) {
    return failure(std::string("R1 StandUp/FSM exception: ") + exception.what());
  } catch (...) {
    return failure("R1 StandUp/FSM unknown exception");
  }
#else
  (void)should_cancel;
  return failure("SDK transport unavailable");
#endif
}

TransportResult SdkTransport::set_velocity(
  double forward, double lateral, double yaw, double duration_sec)
{
#if R1_LIVE_WRITER_HAS_SDK
  if (!impl_->initialized || !impl_->locomotion_enabled || !impl_->loco) {
    return failure("R1 LocoClient is not initialized");
  }
  if (!std::isfinite(forward) || !std::isfinite(lateral) || !std::isfinite(yaw) ||
    !std::isfinite(duration_sec) || duration_sec <= 0.0)
  {
    return failure("R1 SetVelocity rejected nonfinite/invalid input");
  }
  // Mark the command potentially delivered before crossing the RPC boundary.
  // A timeout or nonzero response is ambiguous and must still trigger
  // StopMove from the caller's fail-closed path and again during destruction
  // if that stop does not succeed.
  impl_->velocity_maybe_active = true;
  try {
    const int32_t result = impl_->loco->SetVelocity(
      static_cast<float>(forward), static_cast<float>(lateral),
      static_cast<float>(yaw), static_cast<float>(duration_sec));
    if (result == 0) {
      return success("R1 SetVelocity accepted");
    }
    return failure(
      "R1 SetVelocity ambiguous/nonzero error=" + std::to_string(result));
  } catch (const std::exception & exception) {
    return failure(std::string("R1 SetVelocity ambiguous exception: ") + exception.what());
  } catch (...) {
    return failure("R1 SetVelocity ambiguous unknown exception");
  }
#else
  (void)forward;
  (void)lateral;
  (void)yaw;
  (void)duration_sec;
  return failure("SDK transport unavailable");
#endif
}

TransportResult SdkTransport::stop_locomotion()
{
#if R1_LIVE_WRITER_HAS_SDK
  if (!impl_->initialized || !impl_->locomotion_enabled || !impl_->loco) {
    return failure("R1 LocoClient is not initialized");
  }
  // Do not create or activate a command client merely to send a speculative
  // stop. StopMove is due only after this transport crossed a StandUp/Start or
  // SetVelocity RPC boundary, whether or not the return code proved delivery.
  if (!impl_->prepare_maybe_active && !impl_->velocity_maybe_active) {
    return success("R1 StopMove skipped; no potentially delivered high-level command");
  }
  try {
    const int32_t result = impl_->loco->StopMove();
    if (result == 0) {
      impl_->prepare_maybe_active = false;
      impl_->velocity_maybe_active = false;
      return success("R1 StopMove accepted");
    }
    return failure(
      "R1 StopMove ambiguous/nonzero error=" + std::to_string(result));
  } catch (const std::exception & exception) {
    return failure(std::string("R1 StopMove ambiguous exception: ") + exception.what());
  } catch (...) {
    return failure("R1 StopMove ambiguous unknown exception");
  }
#else
  return failure("SDK transport unavailable");
#endif
}

TransportResult SdkTransport::seed_head(const ArmSdkPositions & positions)
{
  return seed_head_weighted(positions, 1.0);
}

TransportResult SdkTransport::seed_head_weighted(
  const ArmSdkPositions & positions, double weight)
{
#if R1_LIVE_WRITER_HAS_SDK
  if (!impl_->initialized || !impl_->head_enabled || !impl_->arm_sdk) {
    return failure("R1 ArmSdk is not initialized");
  }
  if (!finite_head(positions)) {
    return failure("R1 ArmSdk seed rejected nonfinite positions");
  }
  if (!valid_arm_weight(weight)) {
    return failure("R1 ArmSdk seed rejected weight outside finite [0,1]");
  }
  // unlockAndPublish() is a one-way publisher boundary. Mark the frame as
  // potentially active before a positive-weight crossing and retain a finite
  // pose for a direct weight-zero cleanup if the call returns ambiguously.
  impl_->last_head = positions;
  const float sdk_weight = static_cast<float>(weight);
  if (sdk_weight > 0.0F) {
    impl_->head_maybe_active = true;
    impl_->head_release_weight = sdk_weight;
    impl_->head_weight_delivery_ambiguous = true;
  } else if (impl_->head_maybe_active) {
    impl_->head_weight_delivery_ambiguous = true;
  }
  const TransportResult result = impl_->publish_arm(positions, sdk_weight);
  if (result.ok) {
    impl_->head_seeded = true;
    impl_->head_release_weight = sdk_weight;
    impl_->head_weight_delivery_ambiguous = false;
    return success(
      "R1 ArmSdk seeded all 13 fields from fresh JointState at weight=" +
      std::to_string(weight));
  }
  return result;
#else
  (void)positions;
  (void)weight;
  return failure("SDK transport unavailable");
#endif
}

TransportResult SdkTransport::command_head(const ArmSdkPositions & positions)
{
  return command_head_weighted(positions, 1.0);
}

TransportResult SdkTransport::command_head_weighted(
  const ArmSdkPositions & positions, double weight)
{
#if R1_LIVE_WRITER_HAS_SDK
  if (!impl_->head_seeded) {
    return failure("R1 ArmSdk command rejected before 13-field seed");
  }
  if (!finite_head(positions)) {
    return failure("R1 ArmSdk command rejected nonfinite positions");
  }
  if (!valid_arm_weight(weight)) {
    return failure("R1 ArmSdk command rejected weight outside finite [0,1]");
  }

  // A positive frame may acquire ownership even when the publisher returns
  // ambiguously. Record cleanup debt before crossing that one-way boundary.
  // A zero-weight frame cannot create debt, but it also cannot clear existing
  // debt until release_head confirms a final zero and waits a frame period.
  impl_->last_head = positions;
  const float sdk_weight = static_cast<float>(weight);
  if (sdk_weight > 0.0F) {
    impl_->head_maybe_active = true;
    impl_->head_release_weight = sdk_weight;
    impl_->head_weight_delivery_ambiguous = true;
  } else if (impl_->head_maybe_active) {
    impl_->head_weight_delivery_ambiguous = true;
  }
  const TransportResult result = impl_->publish_arm(positions, sdk_weight);
  if (result.ok) {
    impl_->head_release_weight = sdk_weight;
    impl_->head_weight_delivery_ambiguous = false;
  }
  return result;
#else
  (void)positions;
  (void)weight;
  return failure("SDK transport unavailable");
#endif
}

TransportResult SdkTransport::hold_head(const ArmSdkPositions & positions)
{
#if R1_LIVE_WRITER_HAS_SDK
  if (!impl_->head_seeded) {
    if (impl_->head_maybe_active) {
      return success("R1 ArmSdk hold skipped; initial seed delivery is ambiguous");
    }
    return failure("R1 ArmSdk hold skipped; head was never seeded");
  }
  if (impl_->head_weight_delivery_ambiguous) {
    return success("R1 ArmSdk hold skipped; weighted delivery is ambiguous");
  }
  // A fail-closed hold preserves the last confirmed/requested ownership
  // weight. In particular, safe_stop during a partial ownership probe must
  // never turn a 0 < weight < 1 claim into a full-weight claim before release.
  return command_head_weighted(positions, impl_->head_release_weight);
#else
  (void)positions;
  return failure("SDK transport unavailable");
#endif
}

TransportResult SdkTransport::release_head(const ArmSdkPositions & positions)
{
#if R1_LIVE_WRITER_HAS_SDK
  if (!impl_->head_seeded && !impl_->head_maybe_active) {
    return success("R1 ArmSdk release skipped; head was never seeded");
  }
  if (!finite_head(positions)) {
    return failure("R1 ArmSdk release rejected nonfinite hold pose");
  }
  constexpr auto kReleasePeriod = std::chrono::milliseconds(20);
  constexpr int kZeroBurstFrames = 15;
  // The one-second ramp has its own 1.3 s deadline. Even if every ArmSdk lock
  // consumes its full 50 ms timeout, the mandatory 15-frame zero burst keeps
  // the complete release attempt bounded by this independent deadline.
  constexpr auto kReleaseOverallDeadline = std::chrono::milliseconds(2700);
  const auto release_started = std::chrono::steady_clock::now();
  const auto overall_deadline = release_started + kReleaseOverallDeadline;
  const auto publish_zero_burst = [&]() -> TransportResult {
      // Until the complete burst is accepted, any cleanup retry must remain
      // on the direct-zero path and must never restart a positive ramp.
      impl_->head_weight_delivery_ambiguous = true;
      for (int frame = 0; frame < kZeroBurstFrames; ++frame) {
        if (std::chrono::steady_clock::now() >= overall_deadline) {
          return failure(
            "R1 ArmSdk zero-weight burst exceeded overall release deadline");
        }
        const TransportResult zero = impl_->publish_arm(positions, 0.0F);
        if (!zero.ok) {
          return failure(
            "R1 ArmSdk zero-weight burst failed at frame=" +
            std::to_string(frame + 1) + ": " + zero.detail);
        }
        impl_->head_release_weight = 0.0F;
        const auto consume_until = std::chrono::steady_clock::now() + kReleasePeriod;
        if (consume_until > overall_deadline) {
          return failure(
            "R1 ArmSdk zero-weight burst lacks bounded publisher-consumption window");
        }
        std::this_thread::sleep_until(consume_until);
      }
      return success("R1 ArmSdk zero-weight burst queued and consumed");
    };
  if (!impl_->head_seeded || impl_->head_weight_delivery_ambiguous ||
    impl_->head_release_weight <= 0.0F)
  {
    // A passive seed, an ambiguous weighted frame, or an already-requested
    // zero must never be followed by an upward ownership ramp. Publish only a
    // bounded zero-weight burst before clearing cleanup debt.
    const TransportResult direct_zero = publish_zero_burst();
    if (!direct_zero.ok) {
      return failure(
        "R1 ArmSdk direct weight=0 release burst failed: " + direct_zero.detail);
    }
    impl_->head_seeded = false;
    impl_->head_maybe_active = false;
    impl_->head_release_weight = 0.0F;
    impl_->head_weight_delivery_ambiguous = false;
    return success("R1 ArmSdk ownership relinquished directly to weight=0");
  }

  // Follow the R1 vendor example's approximately one-second linear release.
  // publish_arm rewrites q/kp/kd/dq/tau for every one of the 13 fields on
  // every frame.  The independent deadline prevents teardown from blocking
  // indefinitely if the scheduler falls behind.
  constexpr auto kReleaseDuration = std::chrono::milliseconds(1000);
  constexpr int kReleaseSteps = 50;
  constexpr auto kReleaseDeadline = std::chrono::milliseconds(1300);
  const auto started = release_started;
  const auto deadline = started + kReleaseDeadline;
  const float release_start_weight = impl_->head_release_weight;
  TransportResult ramp_result = success("R1 ArmSdk release ramp complete");
  bool published_zero = false;
  for (int step = 1; step <= kReleaseSteps; ++step) {
    if (std::chrono::steady_clock::now() >= deadline) {
      ramp_result = failure("R1 ArmSdk release deadline exceeded");
      impl_->head_weight_delivery_ambiguous = true;
      break;
    }
    const float weight = static_cast<float>(detail::descending_arm_release_weight(
        release_start_weight, static_cast<std::size_t>(step), kReleaseSteps));
    ramp_result = impl_->publish_arm(positions, weight);
    if (!ramp_result.ok) {
      impl_->head_weight_delivery_ambiguous = true;
      break;
    }
    impl_->head_release_weight = weight;
    published_zero = step == kReleaseSteps;
    const auto scheduled = started + std::min(
      kReleaseDuration,
      std::chrono::duration_cast<std::chrono::milliseconds>(kReleasePeriod * step));
    std::this_thread::sleep_until(std::min(scheduled, deadline));
  }

  if (!published_zero) {
    // Make one bounded final relinquish attempt even after timing drift or an
    // intermediate lock failure. publish_arm has its own 50 ms lock timeout.
    impl_->head_weight_delivery_ambiguous = true;
    const TransportResult final_zero = impl_->publish_arm(positions, 0.0F);
    if (!final_zero.ok) {
      return failure(
        ramp_result.detail + "; final weight=0 attempt failed: " + final_zero.detail);
    }
    impl_->head_release_weight = 0.0F;
    published_zero = true;
  }
  // unlockAndPublish queues work for ArmSdk's publisher thread. Repeat zero
  // for a bounded 300 ms burst after both the normal and degraded ramp paths;
  // cleanup debt remains latched unless every zero frame is accepted and gets
  // one publisher-consumption period.
  const TransportResult zero_burst = publish_zero_burst();
  if (!zero_burst.ok) {
    return failure(zero_burst.detail);
  }
  if (!ramp_result.ok) {
    impl_->head_seeded = false;
    impl_->head_maybe_active = false;
    impl_->head_release_weight = 0.0F;
    impl_->head_weight_delivery_ambiguous = false;
    // The smooth ramp degraded, but the same publisher boundary used by a
    // normal release did accept the final zero-weight frame.  Ownership is no
    // longer pending; report successful cleanup with an explicit warning in
    // the detail instead of forcing a retry against an already-released SDK.
    return success(
      "R1 ArmSdk ownership released to weight=0 after degraded ramp: " +
      ramp_result.detail);
  }
  impl_->head_seeded = false;
  impl_->head_maybe_active = false;
  impl_->head_release_weight = 0.0F;
  impl_->head_weight_delivery_ambiguous = false;
  return success("R1 ArmSdk ownership released to weight=0");
#else
  (void)positions;
  return failure("SDK transport unavailable");
#endif
}

bool SdkTransport::initialized() const
{
  return impl_->initialized;
}

}  // namespace r1_live_writer
