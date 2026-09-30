// Copyright 2026
//
// Bounded, read-only observer for Unitree's official wireless-controller
// channel.  This executable creates one typed DDS reader.  It contains no
// publisher, RPC client, locomotion API, or motor-control API.

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#if R1_WIRELESS_CONTROLLER_OBSERVER_HAS_SDK
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
constexpr int kExitSamples = 0;
constexpr int kExitMatchedSilent = 10;
constexpr int kExitUnmatched = 11;
constexpr int kExitUnavailable = 20;
constexpr int kExitRuntimeError = 21;
constexpr int kExitUsage = 64;

struct Options
{
  std::string interface;
  double duration_sec{6.0};
};

void usage(const char * executable)
{
  std::printf(
    "Usage: %s --interface NAME [--duration-sec SEC]\n"
    "\n"
    "Read only rt/wirelesscontroller for a bounded interval and report "
    "typed publisher discovery, sample rate, axes, and key bits. No DDS "
    "writer, RPC client, locomotion call, or command API is present.\n"
    "\n"
    "Options:\n"
    "  --interface NAME   Robot-facing Ethernet interface\n"
    "  --duration-sec SEC Observation time, 0.5..30.0 (default 6.0)\n"
    "  -h, --help         Show this help without initializing SDK/DDS\n"
    "\n"
    "Exit status: 0=samples observed; 10=typed publisher matched but silent; "
    "11=no typed publisher matched; 20=SDK unavailable; 21=runtime/ABI "
    "failure; 64=usage error.\n",
    executable);
}

double parse_double(const char * text, const char * option)
{
  char * end = nullptr;
  const double value = std::strtod(text, &end);
  if (text == end || end == nullptr || *end != '\0' || !std::isfinite(value)) {
    throw std::invalid_argument(std::string("invalid value for ") + option);
  }
  return value;
}

Options parse_options(const int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      usage(argv[0]);
      std::exit(kExitSamples);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const char * const value = argv[++index];
    if (argument == "--interface") {
      options.interface = value;
    } else if (argument == "--duration-sec") {
      options.duration_sec = parse_double(value, "--duration-sec");
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
  if (options.duration_sec < 0.5 || options.duration_sec > 30.0) {
    throw std::invalid_argument("--duration-sec must be within 0.5..30.0");
  }
  return options;
}

#if R1_WIRELESS_CONTROLLER_OBSERVER_HAS_SDK

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
    R1_WIRELESS_CONTROLLER_OBSERVER_VENDOR_DDS_DIR,
    "compiled vendor DDS directory");
  if (ddsc.parent_path() != expected || ddscxx.parent_path() != expected) {
    throw std::runtime_error(
            "DDS ABI guard rejected loaded libraries: expected=" + expected.string() +
            " libddsc=" + ddsc.string() + " libddscxx=" + ddscxx.string());
  }
}

struct Range
{
  double minimum{std::numeric_limits<double>::infinity()};
  double maximum{-std::numeric_limits<double>::infinity()};
  double latest{std::numeric_limits<double>::quiet_NaN()};

  void add(const double value)
  {
    minimum = std::min(minimum, value);
    maximum = std::max(maximum, value);
    latest = value;
  }
};

struct Observation
{
  std::mutex mutex;
  std::uint64_t samples{0};
  std::uint64_t nonfinite_samples{0};
  std::uint64_t key_transitions{0};
  bool have_keys{false};
  std::uint16_t latest_keys{0};
  std::uint16_t keys_or{0};
  Clock::time_point first{};
  Clock::time_point last{};
  Range lx;
  Range ly;
  Range rx;
  Range ry;
};

struct Snapshot
{
  std::uint64_t samples{0};
  std::uint64_t nonfinite_samples{0};
  std::uint64_t key_transitions{0};
  std::uint16_t latest_keys{0};
  std::uint16_t keys_or{0};
  Clock::time_point first{};
  Clock::time_point last{};
  Range lx;
  Range ly;
  Range rx;
  Range ry;
};

void on_sample(Observation & observation, const void * raw)
{
  if (raw == nullptr) {
    return;
  }
  const auto & message =
    *static_cast<const unitree_go::msg::dds_::WirelessController_ *>(raw);
  const double lx = message.lx();
  const double ly = message.ly();
  const double rx = message.rx();
  const double ry = message.ry();
  const std::uint16_t keys = message.keys();
  const bool finite =
    std::isfinite(lx) && std::isfinite(ly) && std::isfinite(rx) && std::isfinite(ry);
  const auto now = Clock::now();

  std::lock_guard<std::mutex> lock(observation.mutex);
  if (observation.samples == 0) {
    observation.first = now;
  }
  observation.last = now;
  ++observation.samples;
  if (!finite) {
    ++observation.nonfinite_samples;
  } else {
    observation.lx.add(lx);
    observation.ly.add(ly);
    observation.rx.add(rx);
    observation.ry.add(ry);
  }
  if (observation.have_keys && observation.latest_keys != keys) {
    ++observation.key_transitions;
  }
  observation.have_keys = true;
  observation.latest_keys = keys;
  observation.keys_or = static_cast<std::uint16_t>(observation.keys_or | keys);
}

Snapshot snapshot(Observation & observation)
{
  std::lock_guard<std::mutex> lock(observation.mutex);
  Snapshot result;
  result.samples = observation.samples;
  result.nonfinite_samples = observation.nonfinite_samples;
  result.key_transitions = observation.key_transitions;
  result.latest_keys = observation.latest_keys;
  result.keys_or = observation.keys_or;
  result.first = observation.first;
  result.last = observation.last;
  result.lx = observation.lx;
  result.ly = observation.ly;
  result.rx = observation.rx;
  result.ry = observation.ry;
  return result;
}

void print_range(const char * name, const Range & range)
{
  if (!std::isfinite(range.latest)) {
    std::printf(" %s_last=nan %s_min=nan %s_max=nan", name, name, name);
    return;
  }
  std::printf(
    " %s_last=%.6f %s_min=%.6f %s_max=%.6f",
    name, range.latest, name, range.minimum, name, range.maximum);
}

int observe(const Options & options)
{
  const double hard_timeout_sec = options.duration_sec + 5.0;
  std::thread(
    [hard_timeout_sec]() {
      std::this_thread::sleep_for(std::chrono::duration<double>(hard_timeout_sec));
      std::fprintf(stderr, "WIRELESS_CONTROLLER status=ERROR detail=hard_timeout\n");
      std::fflush(stderr);
      std::_Exit(kExitRuntimeError);
    }).detach();

  if (if_nametoindex(options.interface.c_str()) == 0) {
    throw std::runtime_error("network interface does not exist: " + options.interface);
  }
  verify_vendor_dds_pair();

  auto observation = std::make_shared<Observation>();
  unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);
  auto channel = unitree::robot::ChannelFactory::Instance()->
    CreateRecvChannel<unitree_go::msg::dds_::WirelessController_>(
    kTopic, [observation](const void * raw) {on_sample(*observation, raw);}, 0);
  if (!channel || !channel->GetReader()) {
    throw std::runtime_error("typed rt/wirelesscontroller reader was not created");
  }

  std::int32_t current_publishers = 0;
  std::int32_t total_publishers = 0;
  std::int32_t max_current_publishers = 0;
  std::int32_t max_total_publishers = 0;
  std::size_t handle_count = 0;
  bool metadata_valid = false;
  const char * const expected_type = org::eclipse::cyclonedds::topic::
    TopicTraits<unitree_go::msg::dds_::WirelessController_>::getTypeName();

  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(options.duration_sec));
  {
    auto native_reader = channel->GetReader()->GetNative();
    const dds_entity_t reader_entity = native_reader.delegate()->get_ddsc_entity();
    while (Clock::now() < deadline) {
      dds_subscription_matched_status_t status{};
      const dds_return_t status_result =
        dds_get_subscription_matched_status(reader_entity, &status);
      if (status_result < 0) {
        throw std::runtime_error(
                std::string("dds_get_subscription_matched_status: ") +
                dds_strretcode(status_result));
      }
      current_publishers = status.current_count;
      total_publishers = status.total_count;
      max_current_publishers = std::max(max_current_publishers, current_publishers);
      max_total_publishers = std::max(max_total_publishers, total_publishers);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::array<dds_instance_handle_t, 8> handles{};
    const dds_return_t handle_result = dds_get_matched_publications(
      reader_entity, handles.data(), handles.size());
    if (handle_result < 0) {
      throw std::runtime_error(
              std::string("dds_get_matched_publications: ") +
              dds_strretcode(handle_result));
    }
    handle_count = static_cast<std::size_t>(handle_result);
    metadata_valid = handle_count > 0 && handle_count <= handles.size();
    for (std::size_t index = 0; index < std::min(handle_count, handles.size()); ++index) {
      std::unique_ptr<dds_builtintopic_endpoint_t, decltype(&dds_builtintopic_free_endpoint)>
      publication(
        dds_get_matched_publication_data(reader_entity, handles[index]),
        &dds_builtintopic_free_endpoint);
      if (!publication || publication->topic_name == nullptr ||
        publication->type_name == nullptr ||
        std::strcmp(publication->topic_name, kTopic) != 0 ||
        std::strcmp(publication->type_name, expected_type) != 0)
      {
        metadata_valid = false;
      }
    }
  }
  const auto finished = Clock::now();
  channel.reset();
  const Snapshot result = snapshot(*observation);

  const bool matched = current_publishers > 0 && handle_count > 0 && metadata_valid;
  const char * const status = result.samples > 0 ? "SAMPLES" :
    (matched ? "MATCHED_SILENT" : "UNMATCHED");
  const double span_sec = result.samples > 1 ?
    std::chrono::duration<double>(result.last - result.first).count() : 0.0;
  const double rate_hz = span_sec > 0.0 ?
    static_cast<double>(result.samples - 1) / span_sec : 0.0;
  const double last_age_sec = result.samples > 0 ?
    std::chrono::duration<double>(finished - result.last).count() : -1.0;

  std::printf(
    "WIRELESS_CONTROLLER status=%s topic=%s expected_type=%s "
    "current_publishers=%d handle_count=%zu total_publishers=%d "
    "max_current_publishers=%d max_total_publishers=%d metadata_valid=%s "
    "samples=%llu rate_hz=%.3f span_sec=%.3f last_age_sec=%.3f "
    "nonfinite_samples=%llu keys_latest=0x%04x keys_or=0x%04x "
    "key_transitions=%llu",
    status, kTopic, expected_type, current_publishers, handle_count,
    total_publishers, max_current_publishers, max_total_publishers,
    metadata_valid ? "true" : "false",
    static_cast<unsigned long long>(result.samples), rate_hz, span_sec,
    last_age_sec, static_cast<unsigned long long>(result.nonfinite_samples),
    static_cast<unsigned int>(result.latest_keys),
    static_cast<unsigned int>(result.keys_or),
    static_cast<unsigned long long>(result.key_transitions));
  print_range("lx", result.lx);
  print_range("ly", result.ly);
  print_range("rx", result.rx);
  print_range("ry", result.ry);
  std::printf("\n");

  if (result.samples > 0) {
    return kExitSamples;
  }
  return matched ? kExitMatchedSilent : kExitUnmatched;
}

#endif

}  // namespace

int main(const int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
#if R1_WIRELESS_CONTROLLER_OBSERVER_HAS_SDK
    return observe(options);
#else
    (void)options;
    std::fprintf(stderr, "WIRELESS_CONTROLLER status=UNAVAILABLE detail=sdk_missing\n");
    return kExitUnavailable;
#endif
  } catch (const std::invalid_argument & exception) {
    std::fprintf(stderr, "error: %s\n", exception.what());
    usage(argv[0]);
    return kExitUsage;
  } catch (const std::exception & exception) {
    std::fprintf(
      stderr, "WIRELESS_CONTROLLER status=ERROR detail=%s\n", exception.what());
    return kExitRuntimeError;
  }
}
