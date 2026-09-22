#pragma once

#include <array>
#include <cstddef>
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
};

namespace detail
{

// Pure schedule primitive shared by the SDK release path and unit tests.
// Invalid inputs return NaN so a caller cannot silently invent a safe weight.
double descending_arm_release_weight(
  double start_weight, std::size_t completed_steps, std::size_t total_steps);

}  // namespace detail

class RobotTransport
{
public:
  virtual ~RobotTransport() = default;
  virtual TransportResult initialize(
    const std::string & network_interface, bool head, bool locomotion) = 0;
  virtual TransportResult prepare(
    const CancelCheck & should_cancel = {}, bool enter_locomotion = true) = 0;
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
  virtual bool initialized() const = 0;
};

class MockTransport final : public RobotTransport
{
public:
  TransportResult initialize(
    const std::string & network_interface, bool head, bool locomotion) override;
  TransportResult prepare(
    const CancelCheck & should_cancel = {}, bool enter_locomotion = true) override;
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
  SdkTransport();
  ~SdkTransport() override;
  SdkTransport(const SdkTransport &) = delete;
  SdkTransport & operator=(const SdkTransport &) = delete;

  TransportResult initialize(
    const std::string & network_interface, bool head, bool locomotion) override;
  TransportResult prepare(
    const CancelCheck & should_cancel = {}, bool enter_locomotion = true) override;
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
  bool initialized() const override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace r1_live_writer
