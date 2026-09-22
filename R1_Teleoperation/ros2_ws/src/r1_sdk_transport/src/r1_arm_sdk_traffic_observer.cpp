// Copyright 2026
//
// Bounded, read-only observer for the R1 high-level arm command channel.
// This executable deliberately creates only typed Unitree receive channels and
// no publisher, RPC client, service client, or motor-control API.

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
#include <vector>

#if R1_ARM_SDK_OBSERVER_HAS_SDK
#include <dds/dds.h>
#include <dlfcn.h>
#include <link.h>
#include <net/if.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/common/json/jsonize.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#endif

namespace
{

using Clock = std::chrono::steady_clock;

constexpr char kChannel[] = "rt/arm_sdk";
constexpr char kActionStateChannel[] = "rt/arm/action/state";
constexpr std::size_t kHeadPitchSlot = 29;
constexpr std::size_t kHeadYawSlot = 30;
constexpr double kRequiredMatchStableSec = 2.0;
constexpr auto kMatchPollPeriod = std::chrono::milliseconds(50);

constexpr int kExitClear = 0;
constexpr int kExitActiveFresh = 10;
constexpr int kExitTrafficObserved = 11;
constexpr int kExitActionStateMissing = 12;
constexpr int kExitActionStateMalformed = 13;
constexpr int kExitActionStateActive = 14;
constexpr int kExitMatchEvidenceFailed = 15;
constexpr int kExitUnavailable = 20;
constexpr int kExitRuntimeError = 21;
constexpr int kExitUsage = 64;

struct Options
{
  std::string interface;
  double duration_sec{6.0};
  double fresh_sec{0.50};
  double action_fresh_sec{3.50};
  std::uint64_t min_samples{3};
  std::uint64_t expected_writers{1};
};

void usage(const char * executable)
{
  std::printf(
    "Usage: %s --interface NAME [--duration-sec SEC] [--fresh-sec SEC] "
    "[--action-fresh-sec SEC] [--min-samples N] [--expected-writers 1|2]\n"
    "\n"
    "Read only rt/arm_sdk and rt/arm/action/state for a bounded interval. "
    "No command publisher or RPC client is created. Exit 0 additionally "
    "requires the phase-specific typed rt/arm_sdk writer set with the same "
    "handles and no match "
    "churn for the final 2 seconds.\n"
    "Exit status: 0=idle action state and no command samples; "
    "10=fresh active traffic; "
    "11=traffic observed but not fresh at exit; 12=action state missing; "
    "13=action state malformed; 14=action holding/non-idle; "
    "15=typed writer set absent/unexpected/unstable; 20=SDK unavailable; "
    "21=runtime failure; 64=usage error.\n",
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

std::uint64_t parse_count(const char * text, const char * option)
{
  char * end = nullptr;
  const unsigned long long value = std::strtoull(text, &end, 10);
  if (text == end || end == nullptr || *end != '\0') {
    throw std::invalid_argument(std::string("invalid value for ") + option);
  }
  return static_cast<std::uint64_t>(value);
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      usage(argv[0]);
      std::exit(kExitClear);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const char * const value = argv[++index];
    if (argument == "--interface") {
      options.interface = value;
    } else if (argument == "--duration-sec") {
      options.duration_sec = parse_double(value, "--duration-sec");
    } else if (argument == "--fresh-sec") {
      options.fresh_sec = parse_double(value, "--fresh-sec");
    } else if (argument == "--action-fresh-sec") {
      options.action_fresh_sec = parse_double(value, "--action-fresh-sec");
    } else if (argument == "--min-samples") {
      options.min_samples = parse_count(value, "--min-samples");
    } else if (argument == "--expected-writers") {
      options.expected_writers = parse_count(value, "--expected-writers");
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
  if (options.duration_sec < kRequiredMatchStableSec || options.duration_sec > 30.0) {
    throw std::invalid_argument("--duration-sec must be within 2.0..30.0");
  }
  if (options.fresh_sec < 0.05 || options.fresh_sec > options.duration_sec) {
    throw std::invalid_argument("--fresh-sec must be within 0.05..duration-sec");
  }
  if (options.action_fresh_sec < 0.10 || options.action_fresh_sec > 30.0)
  {
    throw std::invalid_argument(
            "--action-fresh-sec must be within 0.10..30.0");
  }
  if (options.min_samples < 2 || options.min_samples > 10000) {
    throw std::invalid_argument("--min-samples must be within 2..10000");
  }
  if (options.expected_writers < 1 || options.expected_writers > 2) {
    throw std::invalid_argument("--expected-writers must be 1 or 2");
  }
  return options;
}

#if R1_ARM_SDK_OBSERVER_HAS_SDK

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
    R1_ARM_SDK_OBSERVER_VENDOR_DDS_DIR, "compiled vendor DDS directory");
  if (ddsc.parent_path() != expected || ddscxx.parent_path() != expected) {
    throw std::runtime_error(
            "DDS ABI guard rejected loaded libraries: expected=" + expected.string() +
            " libddsc=" + ddsc.string() + " libddscxx=" + ddscxx.string());
  }
}

struct ScalarRange
{
  double minimum{std::numeric_limits<double>::infinity()};
  double maximum{-std::numeric_limits<double>::infinity()};
  double latest{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t finite_samples{0};

  void add(const double value)
  {
    if (!std::isfinite(value)) {
      return;
    }
    minimum = std::min(minimum, value);
    maximum = std::max(maximum, value);
    latest = value;
    ++finite_samples;
  }
};

struct Observation
{
  mutable std::mutex mutex;
  std::uint64_t samples{0};
  Clock::time_point first{};
  Clock::time_point last{};
  std::uint8_t mode_pr_min{std::numeric_limits<std::uint8_t>::max()};
  std::uint8_t mode_pr_max{0};
  std::uint8_t mode_machine_min{std::numeric_limits<std::uint8_t>::max()};
  std::uint8_t mode_machine_max{0};
  std::uint64_t invalid_weight_samples{0};
  std::uint64_t nonfinite_head_samples{0};
  ScalarRange head_pitch;
  ScalarRange head_yaw;
};

struct Snapshot
{
  std::uint64_t samples{0};
  Clock::time_point first{};
  Clock::time_point last{};
  std::uint8_t mode_pr_min{0};
  std::uint8_t mode_pr_max{0};
  std::uint8_t mode_machine_min{0};
  std::uint8_t mode_machine_max{0};
  std::uint64_t invalid_weight_samples{0};
  std::uint64_t nonfinite_head_samples{0};
  ScalarRange head_pitch;
  ScalarRange head_yaw;
};

struct ActionStateObservation
{
  mutable std::mutex mutex;
  std::uint64_t samples{0};
  std::uint64_t malformed_samples{0};
  bool any_holding{false};
  bool any_non_idle{false};
  bool latest_valid{false};
  bool latest_holding{false};
  std::int32_t latest_id{0};
  std::string latest_name;
  Clock::time_point last{};
};

struct ActionStateSnapshot
{
  std::uint64_t samples{0};
  std::uint64_t malformed_samples{0};
  bool any_holding{false};
  bool any_non_idle{false};
  bool latest_valid{false};
  bool latest_holding{false};
  std::int32_t latest_id{0};
  std::string latest_name;
  Clock::time_point last{};
};

struct MatchEvidence
{
  std::uint64_t polls{0};
  bool baseline_established{false};
  bool permanent_violation{false};
  bool discovery_query_failed{false};
  bool final_metadata_valid{false};
  std::string first_violation{"none"};
  std::string discovery_error{"none"};
  Clock::time_point baseline_time{};
  std::vector<dds_instance_handle_t> baseline_handles;
  std::string baseline_handle_text{"none"};
  std::int32_t final_current_count{0};
  std::int32_t final_total_count{0};
  std::int32_t max_current_count{0};
  std::int32_t max_total_count{0};
  std::size_t final_handle_count{0};
};

struct MatchDecision
{
  bool accepted{false};
  const char * status{"BLOCKED_UNMATCHED"};
  double stable_sec{0.0};
};

void remember_match_violation(MatchEvidence & evidence, const char * reason)
{
  evidence.permanent_violation = true;
  if (evidence.first_violation == "none") {
    evidence.first_violation = reason;
  }
}

std::string handle_set_text(const std::vector<dds_instance_handle_t> & handles)
{
  std::string text;
  for (const auto handle : handles) {
    if (!text.empty()) {
      text += ',';
    }
    text += std::to_string(handle);
  }
  return text.empty() ? "none" : text;
}

template<typename Message>
void poll_typed_match(
  dds::sub::DataReader<Message> & reader, MatchEvidence & evidence,
  const char * expected_topic, const char * expected_type,
  const std::size_t expected_writers, const Clock::time_point now)
{
  ++evidence.polls;
  try {
    // CycloneDDS 0.10.2's ISO C++ matched_publications() wrapper reports
    // Unsupported even though the corresponding stable C API is present in
    // the same vendor libddsc.  Obtain the entity owned by this *typed* C++
    // DataReader, then query its status/handles through that C API.  No new DDS
    // entity is created here.
    const dds_entity_t reader_entity = reader.delegate()->get_ddsc_entity();
    dds_subscription_matched_status_t status{};
    const dds_return_t status_result =
      dds_get_subscription_matched_status(reader_entity, &status);
    if (status_result < 0) {
      throw std::runtime_error(
              std::string("dds_get_subscription_matched_status: ") +
              dds_strretcode(status_result));
    }

    std::array<dds_instance_handle_t, 8> handles{};
    const dds_return_t handle_result = dds_get_matched_publications(
      reader_entity, handles.data(), handles.size());
    if (handle_result < 0) {
      throw std::runtime_error(
              std::string("dds_get_matched_publications: ") +
              dds_strretcode(handle_result));
    }
    const std::size_t handle_count = static_cast<std::size_t>(handle_result);
    std::vector<dds_instance_handle_t> current_handles;
    current_handles.assign(
      handles.begin(), handles.begin() + std::min(handle_count, handles.size()));
    std::sort(current_handles.begin(), current_handles.end());
    bool metadata_valid = handle_count > 0;
    if (handle_count > handles.size()) {
      metadata_valid = false;
    }
    const std::size_t inspected_handles = std::min(handle_count, handles.size());
    for (std::size_t index = 0; index < inspected_handles; ++index) {
      std::unique_ptr<dds_builtintopic_endpoint_t, decltype(&dds_builtintopic_free_endpoint)>
      publication(
        dds_get_matched_publication_data(reader_entity, handles[index]),
        &dds_builtintopic_free_endpoint);
      if (!publication || publication->topic_name == nullptr ||
        publication->type_name == nullptr ||
        std::strcmp(publication->topic_name, expected_topic) != 0 ||
        std::strcmp(publication->type_name, expected_type) != 0)
      {
        metadata_valid = false;
      }
    }

    evidence.final_current_count = static_cast<std::int32_t>(status.current_count);
    evidence.final_total_count = static_cast<std::int32_t>(status.total_count);
    evidence.final_handle_count = handle_count;
    evidence.final_metadata_valid = metadata_valid;
    evidence.max_current_count = std::max(
      evidence.max_current_count, evidence.final_current_count);
    evidence.max_total_count = std::max(
      evidence.max_total_count, evidence.final_total_count);

    const bool exact_count =
      status.current_count == expected_writers && handle_count == expected_writers;
    const bool status_and_handles_agree =
      status.current_count == handle_count;

    if (!evidence.baseline_established) {
      // Before the full phase-specific set is discovered, 0..N-1 typed
      // endpoints are discovery grace.  Cumulative history reaching N without
      // all N still present proves that a match was lost and therefore blocks.
      if (status.total_count == 0 && status.current_count == 0 && handle_count == 0) {
        return;
      }
      if (status.total_count > expected_writers ||
        status.current_count > expected_writers || handle_count > expected_writers)
      {
        remember_match_violation(evidence, "unexpected_initial_count_or_history");
        return;
      }
      if (!metadata_valid && handle_count > 0) {
        remember_match_violation(evidence, "topic_or_type_metadata_mismatch");
        return;
      }
      // Status and handle enumeration are not atomic, so a newly discovered
      // endpoint may appear in the handle set just after the status snapshot.
      // This is allowed only while cumulative history is still below N.
      if (status.total_count < expected_writers) {
        return;
      }
      if (!status_and_handles_agree) {
        remember_match_violation(evidence, "status_handle_count_disagree");
        return;
      }
      if (!exact_count || status.total_count != expected_writers) {
        remember_match_violation(evidence, "unexpected_initial_count_or_history");
        return;
      }
      if (!metadata_valid) {
        remember_match_violation(evidence, "topic_or_type_metadata_mismatch");
        return;
      }
      evidence.baseline_established = true;
      evidence.baseline_time = now;
      evidence.baseline_handles = current_handles;
      evidence.baseline_handle_text = handle_set_text(current_handles);
      return;
    }

    // Reading SubscriptionMatchedStatus clears its *_change fields.  The
    // initial +N was consumed while establishing the baseline; every later
    // non-zero change is therefore churn, even if a drop/rejoin nets back to
    // expected set between two polls. total_count and handle-set comparison are
    // independent backstops for a fast replacement.
    if (status.current_count_change != 0 || status.total_count_change != 0) {
      remember_match_violation(evidence, "post_baseline_status_change");
    }
    if (!status_and_handles_agree || !exact_count ||
      status.total_count != expected_writers)
    {
      remember_match_violation(evidence, "post_baseline_count_or_history_change");
    }
    if (!metadata_valid) {
      remember_match_violation(evidence, "post_baseline_metadata_mismatch");
    }
    if (handle_count == expected_writers &&
      current_handles != evidence.baseline_handles)
    {
      remember_match_violation(evidence, "publication_handle_changed");
    }
  } catch (const std::exception & exception) {
    evidence.discovery_query_failed = true;
    evidence.discovery_error = exception.what();
    remember_match_violation(evidence, "dds_discovery_query_failed");
  } catch (...) {
    evidence.discovery_query_failed = true;
    evidence.discovery_error = "unknown";
    remember_match_violation(evidence, "dds_discovery_query_failed");
  }
}

MatchDecision decide_match(
  const MatchEvidence & evidence, const Clock::time_point finished)
{
  MatchDecision decision;
  if (evidence.baseline_established) {
    decision.stable_sec = std::max(
      0.0, std::chrono::duration<double>(finished - evidence.baseline_time).count());
  }
  if (evidence.discovery_query_failed) {
    decision.status = "BLOCKED_DISCOVERY_QUERY_ERROR";
  } else if (evidence.permanent_violation) {
    decision.status = "BLOCKED_COUNT_OR_CHURN";
  } else if (!evidence.baseline_established) {
    decision.status = "BLOCKED_UNMATCHED";
  } else if (decision.stable_sec < kRequiredMatchStableSec) {
    decision.status = "BLOCKED_NOT_STABLE";
  } else {
    decision.accepted = true;
    decision.status = "TYPED_STABLE";
  }
  return decision;
}

void on_command(Observation & observation, const void * raw)
{
  if (raw == nullptr) {
    return;
  }
  const auto & command =
    *static_cast<const unitree_hg::msg::dds_::LowCmd_ *>(raw);
  const auto now = Clock::now();
  const std::uint8_t mode_pr = command.mode_pr();
  const std::uint8_t mode_machine = command.mode_machine();
  const double head_pitch = static_cast<double>(
    command.motor_cmd().at(kHeadPitchSlot).q());
  const double head_yaw = static_cast<double>(
    command.motor_cmd().at(kHeadYawSlot).q());

  std::lock_guard<std::mutex> lock(observation.mutex);
  if (observation.samples == 0) {
    observation.first = now;
  }
  observation.last = now;
  ++observation.samples;
  observation.mode_pr_min = std::min(observation.mode_pr_min, mode_pr);
  observation.mode_pr_max = std::max(observation.mode_pr_max, mode_pr);
  observation.mode_machine_min = std::min(observation.mode_machine_min, mode_machine);
  observation.mode_machine_max = std::max(observation.mode_machine_max, mode_machine);
  if (mode_pr > 100) {
    ++observation.invalid_weight_samples;
  }
  if (!std::isfinite(head_pitch) || !std::isfinite(head_yaw)) {
    ++observation.nonfinite_head_samples;
  }
  observation.head_pitch.add(head_pitch);
  observation.head_yaw.add(head_yaw);
}

Snapshot snapshot(const Observation & observation)
{
  std::lock_guard<std::mutex> lock(observation.mutex);
  Snapshot result;
  result.samples = observation.samples;
  result.first = observation.first;
  result.last = observation.last;
  result.mode_pr_min = observation.mode_pr_min;
  result.mode_pr_max = observation.mode_pr_max;
  result.mode_machine_min = observation.mode_machine_min;
  result.mode_machine_max = observation.mode_machine_max;
  result.invalid_weight_samples = observation.invalid_weight_samples;
  result.nonfinite_head_samples = observation.nonfinite_head_samples;
  result.head_pitch = observation.head_pitch;
  result.head_yaw = observation.head_yaw;
  return result;
}

void on_action_state(ActionStateObservation & observation, const void * raw)
{
  const auto now = Clock::now();
  bool valid = false;
  bool holding = false;
  std::int32_t id = 0;
  std::string name;
  try {
    if (raw == nullptr) {
      throw std::runtime_error("null action-state sample");
    }
    const auto & message =
      *static_cast<const std_msgs::msg::dds_::String_ *>(raw);
    const unitree::common::Any parsed =
      unitree::common::FromJsonString(message.data());
    if (!unitree::common::IsJsonMap(parsed)) {
      throw std::runtime_error("action state is not a JSON object");
    }
    const auto & map = unitree::common::AnyCast<unitree::common::JsonMap>(parsed);
    const auto holding_field = map.find("holding");
    const auto id_field = map.find("id");
    const auto name_field = map.find("name");
    if (holding_field == map.end() || id_field == map.end() || name_field == map.end()) {
      throw std::runtime_error("action state lacks holding/id/name");
    }
    unitree::common::FromJson(holding_field->second, holding);
    unitree::common::FromJson(id_field->second, id);
    unitree::common::FromJson(name_field->second, name);
    valid = true;
  } catch (...) {
    valid = false;
  }

  std::lock_guard<std::mutex> lock(observation.mutex);
  ++observation.samples;
  observation.last = now;
  observation.latest_valid = valid;
  if (!valid) {
    ++observation.malformed_samples;
    return;
  }
  observation.latest_holding = holding;
  observation.latest_id = id;
  observation.latest_name = name;
  observation.any_holding = observation.any_holding || holding;
  observation.any_non_idle = observation.any_non_idle ||
    holding || id != 0 || !name.empty();
}

ActionStateSnapshot snapshot(const ActionStateObservation & observation)
{
  std::lock_guard<std::mutex> lock(observation.mutex);
  ActionStateSnapshot result;
  result.samples = observation.samples;
  result.malformed_samples = observation.malformed_samples;
  result.any_holding = observation.any_holding;
  result.any_non_idle = observation.any_non_idle;
  result.latest_valid = observation.latest_valid;
  result.latest_holding = observation.latest_holding;
  result.latest_id = observation.latest_id;
  result.latest_name = observation.latest_name;
  result.last = observation.last;
  return result;
}

void print_range(const char * name, const ScalarRange & range)
{
  if (range.finite_samples == 0) {
    std::printf(" %s_last=nan %s_min=nan %s_max=nan", name, name, name);
    return;
  }
  std::printf(
    " %s_last=%.9f %s_min=%.9f %s_max=%.9f",
    name, range.latest, name, range.minimum, name, range.maximum);
}

int observe(const Options & options)
{
  // Defense in depth around vendor discovery/channel teardown: the official
  // wrapper also uses timeout, but the executable itself cannot outlive its
  // requested observation window by more than five seconds.
  const double hard_timeout_sec = options.duration_sec + 5.0;
  std::thread(
    [hard_timeout_sec]() {
      std::this_thread::sleep_for(std::chrono::duration<double>(hard_timeout_sec));
      std::fprintf(
        stderr, "ARM_SDK_TRAFFIC status=OBSERVER_ERROR detail=hard_timeout\n");
      std::fflush(stderr);
      std::_Exit(kExitRuntimeError);
    }).detach();

  if (if_nametoindex(options.interface.c_str()) == 0) {
    throw std::runtime_error("network interface does not exist: " + options.interface);
  }
  verify_vendor_dds_pair();

  const auto observation = std::make_shared<Observation>();
  const auto action_observation = std::make_shared<ActionStateObservation>();
  unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);
  auto command_channel = unitree::robot::ChannelFactory::Instance()->
    CreateRecvChannel<unitree_hg::msg::dds_::LowCmd_>(
    kChannel,
    // Queue length zero executes the callback synchronously in the DDS reader
    // listener.  A queued callback can still be pending when the channel
    // closes and its vendor listener sets the quit flag; that would make a
    // just-received LowCmd disappear from the final snapshot -- a fail-open
    // race for this gate's "any sample blocks" contract.
    [observation](const void * raw) {on_command(*observation, raw);}, 0);
  auto action_state_channel = unitree::robot::ChannelFactory::Instance()->
    CreateRecvChannel<std_msgs::msg::dds_::String_>(
    kActionStateChannel,
    [action_observation](const void * raw) {
      on_action_state(*action_observation, raw);
    }, 0);

  if (!command_channel || !command_channel->GetReader()) {
    throw std::runtime_error("typed rt/arm_sdk reader was not created");
  }

  MatchEvidence match_evidence;
  const char * const expected_type =
    org::eclipse::cyclonedds::topic::
    TopicTraits<unitree_hg::msg::dds_::LowCmd_>::getTypeName();

  const auto started = Clock::now();
  const auto deadline = started + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(options.duration_sec));
  {
    // Copying this C++ reference does not create another DDS reader.  Keep the
    // copy inside this scope so the Unitree channel and its listener still own
    // the last reference when they are torn down below.
    auto native_reader = command_channel->GetReader()->GetNative();
    while (true) {
      const auto now = Clock::now();
      poll_typed_match(
        native_reader, match_evidence, kChannel, expected_type,
        static_cast<std::size_t>(options.expected_writers), now);
      if (now >= deadline) {
        break;
      }
      std::this_thread::sleep_until(std::min(deadline, now + kMatchPollPeriod));
    }
  }
  const auto finished = Clock::now();
  command_channel.reset();
  action_state_channel.reset();
  const Snapshot result = snapshot(*observation);
  const ActionStateSnapshot action = snapshot(*action_observation);
  const MatchDecision match = decide_match(match_evidence, finished);

  std::printf(
    "ARM_SDK_MATCH status=%s topic=%s expected_type=%s expected_writers=%zu "
    "current_count=%d handle_count=%zu total_count=%d max_current_count=%d "
    "max_total_count=%d baseline_handles=%s stable_sec=%.3f required_stable_sec=%.3f "
    "polls=%llu metadata_valid=%s churn=%s first_violation=%s\n",
    match.status, kChannel, expected_type,
    static_cast<std::size_t>(options.expected_writers),
    match_evidence.final_current_count, match_evidence.final_handle_count,
    match_evidence.final_total_count, match_evidence.max_current_count,
    match_evidence.max_total_count, match_evidence.baseline_handle_text.c_str(),
    match.stable_sec, kRequiredMatchStableSec,
    static_cast<unsigned long long>(match_evidence.polls),
    match_evidence.final_metadata_valid ? "true" : "false",
    match_evidence.permanent_violation ? "true" : "false",
    match_evidence.first_violation.c_str());
  if (match_evidence.discovery_query_failed) {
    std::fprintf(
      stderr, "ARM_SDK_MATCH detail=discovery_query_failed error=%s\n",
      match_evidence.discovery_error.c_str());
  }

  if (result.samples == 0) {
    std::printf(
      "ARM_SDK_TRAFFIC status=CLEAR_NO_SAMPLES topic=%s samples=0 "
      "duration_sec=%.3f\n", kChannel, options.duration_sec);
  } else {
    const double span_sec = result.samples > 1 ?
      std::chrono::duration<double>(result.last - result.first).count() : 0.0;
    const double rate_hz = span_sec > 0.0 ?
      static_cast<double>(result.samples - 1) / span_sec : 0.0;
    const double last_age_sec =
      std::chrono::duration<double>(finished - result.last).count();
    const bool fresh = result.samples >= options.min_samples &&
      last_age_sec <= options.fresh_sec;
    const bool weight_valid = result.invalid_weight_samples == 0;

    std::printf(
      "ARM_SDK_TRAFFIC status=%s topic=%s samples=%llu rate_hz=%.3f "
      "span_sec=%.3f last_age_sec=%.3f mode_pr_min=%u mode_pr_max=%u "
      "weight_encoding=r1_mode_pr_percent weight_valid=%s",
      fresh ? "BLOCKED_ACTIVE_FRESH" : "BLOCKED_OBSERVED_NOT_FRESH",
      kChannel, static_cast<unsigned long long>(result.samples), rate_hz,
      span_sec, last_age_sec, static_cast<unsigned int>(result.mode_pr_min),
      static_cast<unsigned int>(result.mode_pr_max), weight_valid ? "true" : "false");
    if (weight_valid) {
      // R1 r1_pub.h encodes weight as clamp(int(coefficient * 100), 0, 100).
      std::printf(
        " weight_min=%.2f weight_max=%.2f",
        static_cast<double>(result.mode_pr_min) / 100.0,
        static_cast<double>(result.mode_pr_max) / 100.0);
    } else {
      std::printf(" weight_min=nan weight_max=nan");
    }
    std::printf(
      " mode_machine_min=%u mode_machine_max=%u",
      static_cast<unsigned int>(result.mode_machine_min),
      static_cast<unsigned int>(result.mode_machine_max));
    print_range("head_pitch_q_slot29", result.head_pitch);
    print_range("head_yaw_q_slot30", result.head_yaw);
    std::printf(
      " nonfinite_head_samples=%llu invalid_weight_samples=%llu\n",
      static_cast<unsigned long long>(result.nonfinite_head_samples),
      static_cast<unsigned long long>(result.invalid_weight_samples));
  }

  bool action_stale = action.samples == 0;
  if (action.samples == 0) {
    std::printf(
      "ARM_ACTION_STATE status=BLOCKED_MISSING_OR_STALE topic=%s samples=0\n",
      kActionStateChannel);
  } else {
    const double action_last_age_sec =
      std::chrono::duration<double>(finished - action.last).count();
    action_stale = action_last_age_sec > options.action_fresh_sec;
    const char * const action_status = action.malformed_samples > 0 ?
      "BLOCKED_MALFORMED" :
      (action_stale ? "BLOCKED_STALE" :
      (action.any_non_idle ? "BLOCKED_HOLDING_OR_NON_IDLE" : "IDLE_FRESH_BOUNDED"));
    std::printf(
      "ARM_ACTION_STATE status=%s topic=%s samples=%llu malformed=%llu "
      "last_age_sec=%.3f latest_valid=%s latest_holding=%s latest_id=%d "
      "latest_name=%s any_holding=%s any_non_idle=%s\n",
      action_status,
      kActionStateChannel, static_cast<unsigned long long>(action.samples),
      static_cast<unsigned long long>(action.malformed_samples), action_last_age_sec,
      action.latest_valid ? "true" : "false",
      action.latest_holding ? "true" : "false", action.latest_id,
      action.latest_name.empty() ? "<empty>" : action.latest_name.c_str(),
      action.any_holding ? "true" : "false",
      action.any_non_idle ? "true" : "false");
  }

  if (action.malformed_samples > 0 || (action.samples > 0 && !action.latest_valid)) {
    return kExitActionStateMalformed;
  }
  if (action.any_holding || action.any_non_idle) {
    return kExitActionStateActive;
  }
  if (result.samples > 0) {
    const double lowcmd_last_age_sec =
      std::chrono::duration<double>(finished - result.last).count();
    const bool lowcmd_fresh = result.samples >= options.min_samples &&
      lowcmd_last_age_sec <= options.fresh_sec;
    return lowcmd_fresh ? kExitActiveFresh : kExitTrafficObserved;
  }
  if (!match.accepted) {
    return kExitMatchEvidenceFailed;
  }
  return action_stale ? kExitActionStateMissing : kExitClear;
}

#endif

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  try {
    options = parse_options(argc, argv);
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "r1_arm_sdk_traffic_observer usage error: %s\n", exception.what());
    usage(argv[0]);
    return kExitUsage;
  }

#if R1_ARM_SDK_OBSERVER_HAS_SDK
  try {
    return observe(options);
  } catch (const std::exception & exception) {
    std::fprintf(
      stderr, "ARM_SDK_TRAFFIC status=OBSERVER_ERROR detail=%s\n", exception.what());
    return kExitRuntimeError;
  } catch (...) {
    std::fprintf(stderr, "ARM_SDK_TRAFFIC status=OBSERVER_ERROR detail=unknown\n");
    return kExitRuntimeError;
  }
#else
  (void)options;
  std::fprintf(
    stderr, "ARM_SDK_TRAFFIC status=OBSERVER_UNAVAILABLE sdk_compiled=false\n");
  return kExitUnavailable;
#endif
}
