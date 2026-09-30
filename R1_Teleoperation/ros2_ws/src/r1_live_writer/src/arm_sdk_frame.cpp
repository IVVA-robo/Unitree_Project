#include "r1_live_writer/arm_sdk_frame.hpp"

#if R1_LIVE_WRITER_HAS_SDK
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace r1_live_writer
{
namespace
{
static_assert(sizeof(float) == sizeof(std::uint32_t));
static_assert(std::numeric_limits<float>::is_iec559, "HG commands require IEEE754 floats");

std::uint32_t float_word(float value) noexcept
{
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

void add_crc_word(std::uint32_t word, std::uint32_t & crc) noexcept
{
  constexpr std::uint32_t polynomial = 0x04c11db7U;
  for (std::uint32_t bit = 0x80000000U; bit != 0U; bit >>= 1U) {
    const bool high = (crc & 0x80000000U) != 0U;
    crc <<= 1U;
    if (high) {
      crc ^= polynomial;
    }
    if ((word & bit) != 0U) {
      crc ^= polynomial;
    }
  }
}
}  // namespace

std::uint32_t hg_low_cmd_crc(const unitree_hg::msg::dds_::LowCmd_ & frame) noexcept
{
  // SDK2 Python utils/crc.py __packFmtHGLowCmd:
  // '<2B2x' + 'B3x5fI' * 35 + '5I'. Four final reserve words participate;
  // the final crc word does not. Padding is defined as zero, not read from C++.
  std::uint32_t crc = 0xffffffffU;
  add_crc_word(
    static_cast<std::uint32_t>(frame.mode_pr()) |
    (static_cast<std::uint32_t>(frame.mode_machine()) << 8U), crc);
  for (const auto & motor : frame.motor_cmd()) {
    add_crc_word(motor.mode(), crc);
    add_crc_word(float_word(motor.q()), crc);
    add_crc_word(float_word(motor.dq()), crc);
    add_crc_word(float_word(motor.tau()), crc);
    add_crc_word(float_word(motor.kp()), crc);
    add_crc_word(float_word(motor.kd()), crc);
    add_crc_word(motor.reserve(), crc);
  }
  for (const auto reserve : frame.reserve()) {
    add_crc_word(reserve, crc);
  }
  return crc;
}

bool build_r1_a5_arm_sdk_frame(
  const ArmSdkPositions & positions,
  float weight,
  std::uint8_t measured_mode_machine,
  float measured_waist_roll,
  unitree_hg::msg::dds_::LowCmd_ & output) noexcept
{
  if (!std::isfinite(weight) || weight < 0.0F || weight > 1.0F ||
    !std::isfinite(measured_waist_roll))
  {
    return false;
  }
  for (const double position : positions) {
    if (!std::isfinite(position) ||
      std::abs(position) > static_cast<double>(std::numeric_limits<float>::max()))
    {
      return false;
    }
  }

  // R1_A5 motion path pinned at xr_teleoperate 817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b.
  // Fresh construction prevents old leg commands or reserved bits leaking in.
  unitree_hg::msg::dds_::LowCmd_ frame;
  frame.mode_pr(static_cast<std::uint8_t>(weight * 100.0F));
  frame.mode_machine(measured_mode_machine);
  constexpr std::array<std::size_t, 13> indices = {
    15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 13, 29, 30};
  constexpr std::array<float, 13> kp = {
    50, 50, 40, 40, 30, 50, 50, 40, 40, 30, 50, 15, 15};
  constexpr std::array<float, 13> kd = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 1, 1};
  for (std::size_t i = 0; i < indices.size(); ++i) {
    auto & motor = frame.motor_cmd()[indices[i]];
    motor.mode(1);
    motor.q(static_cast<float>(positions[i]));
    motor.kp(kp[i]);
    motor.kd(kd[i]);
  }
  auto & waist_roll = frame.motor_cmd()[12];
  waist_roll.mode(1);
  waist_roll.q(measured_waist_roll);
  waist_roll.kp(50.0F);
  waist_roll.kd(3.0F);
  // dq, tau, reserves and all unaddressed slots retain their fresh zero values.
  frame.crc(hg_low_cmd_crc(frame));
  output = frame;
  return true;
}

}  // namespace r1_live_writer
#endif
