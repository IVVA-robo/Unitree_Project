#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace r1_live_writer
{

using ArmSdkPositions = std::array<double, 13>;
using CancelCheck = std::function<bool()>;

struct TransportResult
{
  bool ok{false};
  std::string detail;
  // Raw vendor RPC status when one exists. Zero is also used for local
  // validation/transport failures that have no vendor status.
  int32_t status_code{0};
};

struct LocoModeSample
{
  bool valid{false};
  int fsm{-1};
  // Request start, not completion: an in-flight pre-release query must never
  // authorize the next owner. All callers use the steady clock.
  std::chrono::steady_clock::time_point requested_at{};
};

namespace detail
{

// Metadata for the opt-in, August 2026 R1 A5 ArmSdk frame. A seed captures
// measured waist roll and machine mode; later positive frames need fresh
// feedback in that same mode. A zero-weight release may reuse the captured
// metadata even after feedback disappears. This class never opens DDS.
class ArmSdkMetadataGate
{
public:
  using Clock = std::chrono::steady_clock;
  void observe(std::uint8_t mode_machine, double waist_roll, Clock::time_point now);
  bool seed(Clock::time_point now);
  bool permits(double weight, Clock::time_point now) const;
  bool prepare_frame(double weight, Clock::time_point now);
  void reset_seed() {seeded_ = false;}
  std::uint8_t mode_machine() const {return seed_mode_;}
  float waist_roll() const {return seed_roll_;}

private:
  bool fresh(Clock::time_point now) const;
  bool observed_{false};
  bool seeded_{false};
  bool positive_requested_{false};
  std::uint8_t latest_mode_{0};
  std::uint8_t seed_mode_{0};
  float latest_roll_{0.0F};
  float seed_roll_{0.0F};
  Clock::time_point arrival_{};
};

struct WirelessControllerCommand
{
  float lx{0.0F};
  float ly{0.0F};
  float rx{0.0F};
  float ry{0.0F};
  std::uint16_t keys{0};
};

// Convert the writer's bounded physical velocity request to the normalized
// joystick axes observed from Unitree Explore on rt/wirelesscontroller.
// Forward is left-stick Y, lateral is left-stick X, and yaw is right-stick X.
WirelessControllerCommand velocity_to_wireless_controller(
  double forward, double lateral, double yaw);

// Pure schedule primitive shared by the SDK release path and unit tests.
// Invalid inputs return NaN so a caller cannot silently invent a safe weight.
double descending_arm_release_weight(
  double start_weight, std::size_t completed_steps, std::size_t total_steps);

// A nonzero StopMove may use the read-only FSM 4 fallback only when no
// velocity-capable boundary was ever crossed by this transport.
bool static_stop_fallback_eligible(
  bool locomotion_mode_maybe_active, bool velocity_maybe_active);
bool confirms_static_standing_sample(int32_t query_result, int fsm_id);
bool no_velocity_stop_fallback_eligible(bool velocity_maybe_active);
bool confirms_nonmoving_mode_sample(
  int32_t query_result, int fsm_id, bool locomotion_mode_maybe_active);

}  // namespace detail

class RobotTransport
{
public:
  virtual ~RobotTransport() = default;
  virtual TransportResult initialize(
    const std::string & network_interface, bool head, bool locomotion) = 0;
  virtual TransportResult prepare(
    const CancelCheck & should_cancel = {}, bool enter_locomotion = true,
    bool retain_stand_cleanup_debt = false, int speed_mode = -1) = 0;
  virtual TransportResult set_velocity(
    double forward, double lateral, double yaw, double duration_sec) = 0;
  virtual TransportResult stop_locomotion() = 0;
  // Legacy head entry points retain full-weight behavior. The weighted seed
  // permits a passive weight-zero initialization; every weighted command
  // still requires one successful seed and a finite weight in [0, 1].
  virtual TransportResult seed_head(const ArmSdkPositions & positions) = 0;
  virtual TransportResult seed_head_weighted(
    const ArmSdkPositions & positions, double weight) = 0;
  virtual TransportResult command_head(const ArmSdkPositions & positions) = 0;
  virtual TransportResult command_head_weighted(
    const ArmSdkPositions & positions, double weight) = 0;
  virtual TransportResult hold_head(const ArmSdkPositions & positions) = 0;
  virtual TransportResult release_head(const ArmSdkPositions & positions) = 0;
  virtual TransportResult finish_head_handover() = 0;
  virtual LocoModeSample poll_loco_mode() = 0;
  virtual bool initialized() const = 0;
};

class MockTransport final : public RobotTransport
{
public:
  TransportResult initialize(
    const std::string & network_interface, bool head, bool locomotion) override;
  TransportResult prepare(
    const CancelCheck & should_cancel = {}, bool enter_locomotion = true,
    bool retain_stand_cleanup_debt = false, int speed_mode = -1) override;
  TransportResult set_velocity(
    double forward, double lateral, double yaw, double duration_sec) override;
  TransportResult stop_locomotion() override;
  TransportResult seed_head(const ArmSdkPositions & positions) override;
  TransportResult seed_head_weighted(
    const ArmSdkPositions & positions, double weight) override;
  TransportResult command_head(const ArmSdkPositions & positions) override;
  TransportResult command_head_weighted(
    const ArmSdkPositions & positions, double weight) override;
  TransportResult hold_head(const ArmSdkPositions & positions) override;
  TransportResult release_head(const ArmSdkPositions & positions) override;
  TransportResult finish_head_handover() override;
  LocoModeSample poll_loco_mode() override;
  bool initialized() const override {return initialized_;}

  std::size_t prepare_calls() const {return prepare_calls_;}
  std::size_t velocity_calls() const {return velocity_calls_;}
  std::size_t stop_calls() const {return stop_calls_;}
  std::size_t head_seed_calls() const {return head_seed_calls_;}
  std::size_t head_command_calls() const {return head_command_calls_;}
  std::size_t head_hold_calls() const {return head_hold_calls_;}
  std::size_t head_release_calls() const {return head_release_calls_;}
  std::array<double, 3> last_velocity() const {return last_velocity_;}
  ArmSdkPositions last_head() const {return last_head_;}
  double last_head_weight() const {return last_head_weight_;}

private:
  bool initialized_{false};
  bool head_enabled_{false};
  bool locomotion_enabled_{false};
  bool head_seeded_{false};
  std::size_t prepare_calls_{0};
  std::size_t velocity_calls_{0};
  std::size_t stop_calls_{0};
  std::size_t head_seed_calls_{0};
  std::size_t head_command_calls_{0};
  std::size_t head_hold_calls_{0};
  std::size_t head_release_calls_{0};
  std::array<double, 3> last_velocity_{};
  ArmSdkPositions last_head_{};
  double last_head_weight_{0.0};
};

// The implementation has no SDK object in its constructor.  initialize() is
// the sole SDK boundary and must only be called after the node's complete live
// authorization gate succeeds.
class SdkTransport final : public RobotTransport
{
public:
  explicit SdkTransport(
    bool wireless_controller_locomotion = false, bool a5_20260803_frame = false);
  ~SdkTransport() override;
  SdkTransport(const SdkTransport &) = delete;
  SdkTransport & operator=(const SdkTransport &) = delete;

  TransportResult initialize(
    const std::string & network_interface, bool head, bool locomotion) override;
  TransportResult prepare(
    const CancelCheck & should_cancel = {}, bool enter_locomotion = true,
    bool retain_stand_cleanup_debt = false, int speed_mode = -1) override;
  TransportResult set_velocity(
    double forward, double lateral, double yaw, double duration_sec) override;
  TransportResult stop_locomotion() override;
  TransportResult seed_head(const ArmSdkPositions & positions) override;
  TransportResult seed_head_weighted(
    const ArmSdkPositions & positions, double weight) override;
  TransportResult command_head(const ArmSdkPositions & positions) override;
  TransportResult command_head_weighted(
    const ArmSdkPositions & positions, double weight) override;
  TransportResult hold_head(const ArmSdkPositions & positions) override;
  TransportResult release_head(const ArmSdkPositions & positions) override;
  TransportResult finish_head_handover() override;
  LocoModeSample poll_loco_mode() override;
  bool initialized() const override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace r1_live_writer
