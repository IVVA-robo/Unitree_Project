#pragma once

#include "r1_live_writer/transport.hpp"

#if R1_LIVE_WRITER_HAS_SDK
#include <unitree/idl/hg/LowCmd_.hpp>

#include <cstdint>

namespace r1_live_writer
{

// Vendor HG checksum, over the explicitly packed 1000-byte payload preceding
// crc. This is neither DDS CDR serialization nor the C++ object's storage.
std::uint32_t hg_low_cmd_crc(const unitree_hg::msg::dds_::LowCmd_ & frame) noexcept;

// Offline frame construction only: never creates a DDS participant/publisher.
// Positions follow ArmSdkPositions: 15..19,22..26,13,29,30. Slot 12 holds a
// measured waist position; this helper does not perform upstream's zero homing.
// The caller must supply fresh measured mode_machine and waist data and enforce
// limits/authorization before publishing. A finite float representable target
// and finite weight in [0,1] are required. Invalid input leaves output untouched.
bool build_r1_a5_arm_sdk_frame(
  const ArmSdkPositions & positions,
  float weight,
  std::uint8_t measured_mode_machine,
  float measured_waist_roll,
  unitree_hg::msg::dds_::LowCmd_ & output) noexcept;

}  // namespace r1_live_writer
#endif
