// Copyright 2026
// Read-only R1 battery monitor. No publisher, RPC client, or command API.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if R1_BATTERY_OBSERVER_HAS_SDK
#include <net/if.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#endif

namespace
{

std::atomic_bool running{true};

void stop_handler(int)
{
  running.store(false);
}

struct Options
{
  std::string interface;
  std::vector<std::string> topics{
    "rt/lf/bmsstate", "rt/bmsstate", "rt/bms_state"};
};

void usage(const char * executable)
{
  std::printf(
    "Usage: %s --interface NAME [--topic DDS_TOPIC ...]\n"
    "Continuously reads the typed Unitree HG BmsState and prints lines like "
    "BATTERY soc=92. This executable contains no DDS writer or command API.\n",
    executable);
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  bool custom_topics = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      usage(argv[0]);
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const std::string value = argv[++index];
    if (argument == "--interface") {
      options.interface = value;
    } else if (argument == "--topic") {
      if (!custom_topics) {
        options.topics.clear();
        custom_topics = true;
      }
      options.topics.push_back(value);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.interface.empty() || options.interface.size() > 15) {
    throw std::invalid_argument("--interface is required and must name an interface");
  }
  if (options.topics.empty() ||
    std::any_of(options.topics.begin(), options.topics.end(), [](const std::string & topic) {
      return topic.rfind("rt/", 0) != 0 || topic.size() > 96;
    }))
  {
    throw std::invalid_argument("battery topics must be non-empty rt/* names");
  }
  return options;
}

#if R1_BATTERY_OBSERVER_HAS_SDK
using Battery = unitree_hg::msg::dds_::BmsState_;
using BatteryChannel = unitree::robot::ChannelPtr<Battery>;

struct SharedState
{
  std::atomic<int> last_soc{-1};
  std::atomic<std::int64_t> last_print_ms{0};
};

std::int64_t now_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void on_battery(const std::shared_ptr<SharedState> & state, const std::string & topic,
  const void * raw)
{
  if (raw == nullptr) {
    return;
  }
  const auto & message = *static_cast<const Battery *>(raw);
  const int soc = static_cast<int>(message.soc());
  if (soc < 0 || soc > 100) {
    return;
  }
  const auto current_ms = now_ms();
  const int previous = state->last_soc.exchange(soc);
  const auto last_print = state->last_print_ms.load();
  if (soc != previous || current_ms - last_print >= 1000) {
    state->last_print_ms.store(current_ms);
    std::printf(
      "BATTERY soc=%d soh=%u current=%d topic=%s\n",
      soc, static_cast<unsigned>(message.soh()), message.current(), topic.c_str());
    std::fflush(stdout);
  }
}

int observe(const Options & options)
{
  if (if_nametoindex(options.interface.c_str()) == 0) {
    throw std::runtime_error("network interface does not exist: " + options.interface);
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);
  auto state = std::make_shared<SharedState>();
  std::vector<BatteryChannel> channels;
  channels.reserve(options.topics.size());
  for (const auto & topic : options.topics) {
    auto channel = unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<Battery>(
      topic, [state, topic](const void * raw) {on_battery(state, topic, raw);}, 0);
    if (!channel || !channel->GetReader()) {
      throw std::runtime_error("could not create typed battery reader for " + topic);
    }
    channels.push_back(std::move(channel));
  }
  std::printf("BATTERY status=SEARCHING topics=%zu\n", channels.size());
  std::fflush(stdout);
  while (running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  channels.clear();
  unitree::robot::ChannelFactory::Instance()->Release();
  return 0;
}
#endif

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);
#if R1_BATTERY_OBSERVER_HAS_SDK
    return observe(options);
#else
    (void)options;
    std::fprintf(stderr, "BATTERY status=UNAVAILABLE detail=unitree_sdk_missing\n");
    return 20;
#endif
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "BATTERY status=ERROR detail=%s\n", exception.what());
    return 21;
  }
}

