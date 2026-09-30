// Copyright 2026
//
// Narrow commissioning probe for the official Unitree wireless-controller
// channel.  The default mode publishes zero axes only.  The separately gated
// live mode can publish one fixed, bounded forward-stick pulse surrounded by
// zero frames.  It cannot express lateral motion, yaw, buttons, mode changes,
// RPC calls, or arbitrary amplitudes/durations.

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#if R1_WIRELESS_CONTROLLER_PROBE_HAS_SDK
#include <dds/dds.h>
#include <dlfcn.h>
#include <link.h>
#include <net/if.h>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#endif

namespace
{

using Clock = std::chrono::steady_clock;

constexpr char kTopic[] = "rt/wirelesscontroller";
constexpr char kLiveAcknowledgement[] = "BOUNDED_R1_FORWARD_STICK_PROBE";
constexpr char kRunningLiveAcknowledgement[] =
  "BOUNDED_R1_RUNNING_AND_FORWARD_STICK_PROBE";
constexpr float kForwardLy = 0.15F;
constexpr std::uint16_t kRunningKeys =
  static_cast<std::uint16_t>((1U << 4U) | (1U << 8U));  // R2 + A
constexpr double kPreZeroSec = 0.50;
constexpr double kRunningKeyPulseSec = 0.20;
constexpr double kRunningSettleSec = 2.00;
constexpr double kPulseSec = 0.40;
constexpr double kPostZeroSec = 1.00;
constexpr double kRateHz = 25.0;
constexpr int kExitOk = 0;
constexpr int kExitUnmatched = 10;
constexpr int kExitWriteFailed = 11;
constexpr int kExitUnavailable = 20;
constexpr int kExitRuntimeError = 21;
constexpr int kExitUsage = 64;

struct Options
{
  std::string interface;
  bool forward_probe{false};
  bool running_forward_probe{false};
  std::string acknowledgement;
};

void usage(const char * executable)
{
  std::printf(
    "Usage: %s --interface NAME [--forward-probe --acknowledge %s | "
    "--running-forward-probe --acknowledge %s]\n"
    "\n"
    "Default: publish zero axes/buttons at 25 Hz for a bounded discovery and "
    "routing check. --forward-probe adds one fixed ly=0.15 pulse for 0.40 s, "
    "with 0.50 s of zeros before and 1.00 s after. The exact acknowledgement "
    "is mandatory for that live pulse. No lateral/yaw/button/mode/RPC command "
    "is available. --running-forward-probe first emits the official short "
    "R2+A Running Mode combination, releases it, waits 2 seconds, and then "
    "uses the same bounded forward pulse.\n"
    "\n"
    "Exit status: 0=completed; 10=no typed subscriber; 11=DDS write failure; "
    "20=SDK unavailable; 21=runtime/ABI failure; 64=usage error.\n",
    executable, kLiveAcknowledgement, kRunningLiveAcknowledgement);
}

Options parse_options(const int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      usage(argv[0]);
      std::exit(kExitOk);
    }
    if (argument == "--forward-probe") {
      options.forward_probe = true;
      continue;
    }
    if (argument == "--running-forward-probe") {
      options.running_forward_probe = true;
      continue;
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const char * const value = argv[++index];
    if (argument == "--interface") {
      options.interface = value;
    } else if (argument == "--acknowledge") {
      options.acknowledgement = value;
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.interface.empty()) {
    throw std::invalid_argument("--interface is required");
  }
  if (options.interface.size() > 15 ||
    !std::all_of(
      options.interface.begin(), options.interface.end(),
      [](const unsigned char character) {
        return std::isalnum(character) || character == '_' || character == '-' ||
               character == '.';
      }))
  {
    throw std::invalid_argument("invalid network interface name");
  }
  if (options.forward_probe && options.running_forward_probe) {
    throw std::invalid_argument("select only one live probe mode");
  }
  if (options.forward_probe && options.acknowledgement != kLiveAcknowledgement) {
    throw std::invalid_argument("invalid --acknowledge value for --forward-probe");
  }
  if (options.running_forward_probe &&
    options.acknowledgement != kRunningLiveAcknowledgement)
  {
    throw std::invalid_argument(
            "invalid --acknowledge value for --running-forward-probe");
  }
  if (!options.forward_probe && !options.running_forward_probe &&
    !options.acknowledgement.empty())
  {
    throw std::invalid_argument("--acknowledge requires a live probe mode");
  }
  return options;
}

#if R1_WIRELESS_CONTROLLER_PROBE_HAS_SDK

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

void verify_vendor_dds_pair()
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
  void * const handle = dlopen("libddscxx.so.0", RTLD_LAZY | RTLD_NOLOAD);
  if (handle == nullptr) {
    const char * const error = dlerror();
    throw std::runtime_error(
            std::string("DDS ABI guard cannot locate loaded libddscxx.so.0: ") +
            (error == nullptr ? "unknown loader error" : error));
  }
  link_map * map = nullptr;
  const int result = dlinfo(handle, RTLD_DI_LINKMAP, &map);
  const std::string ddscxx_path =
    result == 0 && map != nullptr && map->l_name != nullptr ? map->l_name : "";
  dlclose(handle);
  if (result != 0 || ddscxx_path.empty()) {
    throw std::runtime_error("DDS ABI guard cannot resolve loaded libddscxx.so.0");
  }

  const std::filesystem::path ddscxx = canonical_path_or_throw(
    ddscxx_path, "loaded libddscxx.so.0 path");
  const std::filesystem::path expected = canonical_path_or_throw(
    R1_WIRELESS_CONTROLLER_PROBE_VENDOR_DDS_DIR,
    "compiled vendor DDS directory");
  if (ddsc.parent_path() != expected || ddscxx.parent_path() != expected) {
    throw std::runtime_error(
            "DDS ABI guard rejected loaded libraries: expected=" + expected.string() +
            " libddsc=" + ddsc.string() + " libddscxx=" + ddscxx.string());
  }
}

struct MatchResult
{
  std::int32_t current_subscribers{0};
  std::int32_t total_subscribers{0};
  std::size_t handle_count{0};
  bool metadata_valid{false};
};

template<typename Writer>
MatchResult wait_for_typed_subscriber(Writer & native_writer)
{
  MatchResult result;
  const dds_entity_t writer_entity = native_writer.delegate()->get_ddsc_entity();
  const auto deadline = Clock::now() + std::chrono::seconds(2);
  do {
    dds_publication_matched_status_t status{};
    const dds_return_t status_result =
      dds_get_publication_matched_status(writer_entity, &status);
    if (status_result < 0) {
      throw std::runtime_error(
              std::string("dds_get_publication_matched_status: ") +
              dds_strretcode(status_result));
    }
    result.current_subscribers = status.current_count;
    result.total_subscribers = status.total_count;
    if (result.current_subscribers > 0) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } while (Clock::now() < deadline);

  std::array<dds_instance_handle_t, 16> handles{};
  const dds_return_t handle_result = dds_get_matched_subscriptions(
    writer_entity, handles.data(), handles.size());
  if (handle_result < 0) {
    throw std::runtime_error(
            std::string("dds_get_matched_subscriptions: ") +
            dds_strretcode(handle_result));
  }
  result.handle_count = static_cast<std::size_t>(handle_result);
  result.metadata_valid =
    result.handle_count > 0 && result.handle_count <= handles.size();
  const char * const expected_type = org::eclipse::cyclonedds::topic::
    TopicTraits<unitree_go::msg::dds_::WirelessController_>::getTypeName();
  for (std::size_t index = 0;
    index < std::min(result.handle_count, handles.size()); ++index)
  {
    std::unique_ptr<dds_builtintopic_endpoint_t, decltype(&dds_builtintopic_free_endpoint)>
    subscription(
      dds_get_matched_subscription_data(writer_entity, handles[index]),
      &dds_builtintopic_free_endpoint);
    if (!subscription || subscription->topic_name == nullptr ||
      subscription->type_name == nullptr ||
      std::strcmp(subscription->topic_name, kTopic) != 0 ||
      std::strcmp(subscription->type_name, expected_type) != 0)
    {
      result.metadata_valid = false;
    }
  }
  return result;
}

unitree_go::msg::dds_::WirelessController_ message(
  const float ly, const std::uint16_t keys)
{
  // The probe cannot express any mode/button transition.  The only non-zero
  // field available to the caller is the fixed positive left-stick Y pulse.
  return unitree_go::msg::dds_::WirelessController_(0.0F, ly, 0.0F, 0.0F, keys);
}

template<typename Channel>
bool publish_for(
  Channel & channel, const float ly, const std::uint16_t keys,
  const double duration_sec,
  std::uint64_t & writes)
{
  const auto period = std::chrono::duration<double>(1.0 / kRateHz);
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(duration_sec));
  bool success = true;
  do {
    success = channel->Write(message(ly, keys), 0) && success;
    ++writes;
    std::this_thread::sleep_for(period);
  } while (Clock::now() < deadline);
  return success;
}

int run(const Options & options)
{
  if (if_nametoindex(options.interface.c_str()) == 0) {
    throw std::runtime_error("network interface does not exist: " + options.interface);
  }
  verify_vendor_dds_pair();
  unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);
  auto channel = unitree::robot::ChannelFactory::Instance()->
    CreateSendChannel<unitree_go::msg::dds_::WirelessController_>(kTopic);
  if (!channel || !channel->GetWriter()) {
    throw std::runtime_error("typed rt/wirelesscontroller writer was not created");
  }
  auto native_writer = channel->GetWriter()->GetNative();
  const MatchResult match = wait_for_typed_subscriber(native_writer);
  const bool matched = match.current_subscribers > 0 &&
    match.handle_count > 0 && match.metadata_valid;
  std::printf(
    "WIRELESS_CONTROLLER_PROBE_MATCH status=%s topic=%s "
    "current_subscribers=%d handle_count=%zu total_subscribers=%d "
    "metadata_valid=%s\n",
    matched ? "TYPED_MATCH" : "UNMATCHED",
    kTopic, match.current_subscribers, match.handle_count,
    match.total_subscribers, match.metadata_valid ? "true" : "false");
  std::fflush(stdout);
  if (!matched) {
    return kExitUnmatched;
  }

  std::uint64_t zero_writes = 0;
  std::uint64_t running_key_writes = 0;
  std::uint64_t pulse_writes = 0;
  bool success = publish_for(channel, 0.0F, 0U, kPreZeroSec, zero_writes);
  if (options.running_forward_probe) {
    success = publish_for(
      channel, 0.0F, kRunningKeys, kRunningKeyPulseSec,
      running_key_writes) && success;
    success = publish_for(
      channel, 0.0F, 0U, kRunningSettleSec, zero_writes) && success;
  }
  if (options.forward_probe || options.running_forward_probe) {
    success = publish_for(
      channel, kForwardLy, 0U, kPulseSec, pulse_writes) && success;
  }
  success = publish_for(
    channel, 0.0F, 0U, kPostZeroSec, zero_writes) && success;

  const char * const mode = options.running_forward_probe ?
    "RUNNING_AND_BOUNDED_FORWARD" :
    (options.forward_probe ? "BOUNDED_FORWARD" : "ZERO_ONLY");

  std::printf(
    "WIRELESS_CONTROLLER_PROBE status=%s mode=%s ly=%.3f "
    "pulse_sec=%.3f rate_hz=%.1f running_keys=0x%04x "
    "running_key_writes=%llu pulse_writes=%llu zero_writes=%llu\n",
    success ? "COMPLETE" : "WRITE_FAILED",
    mode,
    (options.forward_probe || options.running_forward_probe) ?
    static_cast<double>(kForwardLy) : 0.0,
    (options.forward_probe || options.running_forward_probe) ? kPulseSec : 0.0,
    kRateHz, static_cast<unsigned int>(kRunningKeys),
    static_cast<unsigned long long>(running_key_writes),
    static_cast<unsigned long long>(pulse_writes),
    static_cast<unsigned long long>(zero_writes));
  return success ? kExitOk : kExitWriteFailed;
}

#endif

}  // namespace

int main(const int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
#if R1_WIRELESS_CONTROLLER_PROBE_HAS_SDK
    return run(options);
#else
    (void)options;
    std::fprintf(
      stderr, "WIRELESS_CONTROLLER_PROBE status=UNAVAILABLE detail=sdk_missing\n");
    return kExitUnavailable;
#endif
  } catch (const std::invalid_argument & exception) {
    std::fprintf(stderr, "error: %s\n", exception.what());
    usage(argv[0]);
    return kExitUsage;
  } catch (const std::exception & exception) {
    std::fprintf(
      stderr, "WIRELESS_CONTROLLER_PROBE status=ERROR detail=%s\n",
      exception.what());
    return kExitRuntimeError;
  }
}
