// Copyright 2026
// Read-only observer for Unitree application RPC request topics.
// This executable intentionally contains no publisher, RPC client, or command API.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if R1_APP_COMMAND_OBSERVER_HAS_SDK
#include <net/if.h>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/internal/internal_idl_decl/Request_.hpp>
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
  std::string output;
  std::string label;
  double duration_sec{12.0};
  std::vector<std::string> topics{
    "rt/api/sport/request",
    "rt/api/motion_switcher/request",
    "rt/api/robot_state/request",
    "rt/api/audiohub/request"};
};

void usage(const char * executable)
{
  std::printf(
    "Usage: %s --interface NAME --output FILE --label TEXT "
    "[--duration-sec SEC] [--topic DDS_TOPIC ...]\n"
    "Read-only capture of typed unitree_api Request messages to JSONL. "
    "No DDS writer, RPC client, locomotion call, or command API is present.\n",
    executable);
}

double parse_double(const char * text, const char * option)
{
  char * end = nullptr;
  const double value = std::strtod(text, &end);
  if (text == end || end == nullptr || *end != '\0') {
    throw std::invalid_argument(std::string("invalid value for ") + option);
  }
  return value;
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
    } else if (argument == "--output") {
      options.output = value;
    } else if (argument == "--label") {
      options.label = value;
    } else if (argument == "--duration-sec") {
      options.duration_sec = parse_double(value.c_str(), "--duration-sec");
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
  if (options.output.empty() || options.label.empty()) {
    throw std::invalid_argument("--output and --label are required");
  }
  if (options.duration_sec < 1.0 || options.duration_sec > 60.0) {
    throw std::invalid_argument("--duration-sec must be within 1..60");
  }
  if (options.topics.empty() ||
    std::any_of(options.topics.begin(), options.topics.end(), [](const std::string & topic) {
      return topic.rfind("rt/api/", 0) != 0 || topic.size() > 128;
    }))
  {
    throw std::invalid_argument("capture topics must be non-empty rt/api/* names");
  }
  return options;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  return output.str();
}

std::string timestamp_utc()
{
  const auto now = std::chrono::system_clock::now();
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  const std::time_t value = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&value, &utc);
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setw(3) << std::setfill('0') << milliseconds.count() << 'Z';
  return output.str();
}

std::string service_from_topic(const std::string & topic)
{
  constexpr const char prefix[] = "rt/api/";
  const auto begin = topic.find(prefix);
  const auto request = topic.rfind("/request");
  if (begin == 0 && request != std::string::npos) {
    return topic.substr(sizeof(prefix) - 1, request - (sizeof(prefix) - 1));
  }
  return topic;
}

#if R1_APP_COMMAND_OBSERVER_HAS_SDK
using Request = unitree_api::msg::dds_::Request_;
using RequestChannel = unitree::robot::ChannelPtr<Request>;

struct Capture
{
  std::mutex mutex;
  std::ofstream output;
  std::string label;
  std::atomic<unsigned long long> samples{0};
};

void on_request(
  const std::shared_ptr<Capture> & capture,
  const std::string & topic,
  const void * raw)
{
  if (raw == nullptr) {
    return;
  }
  const auto & message = *static_cast<const Request *>(raw);
  const auto & identity = message.header().identity();
  std::lock_guard<std::mutex> lock(capture->mutex);
  capture->output
    << "{\"timestamp\":\"" << timestamp_utc()
    << "\",\"label\":\"" << json_escape(capture->label)
    << "\",\"topic\":\"" << json_escape(topic)
    << "\",\"service\":\"" << json_escape(service_from_topic(topic))
    << "\",\"api_id\":" << identity.api_id()
    << ",\"request_id\":" << identity.id()
    << ",\"parameter\":\"" << json_escape(message.parameter())
    << "\",\"binary_size\":" << message.binary().size()
    << "}\n";
  capture->output.flush();
  const auto count = ++capture->samples;
  std::printf(
    "CAPTURE sample=%llu service=%s api_id=%lld\n",
    count, service_from_topic(topic).c_str(),
    static_cast<long long>(identity.api_id()));
  std::fflush(stdout);
}

int observe(const Options & options)
{
  if (if_nametoindex(options.interface.c_str()) == 0) {
    throw std::runtime_error("network interface does not exist: " + options.interface);
  }
  auto capture = std::make_shared<Capture>();
  capture->label = options.label;
  capture->output.open(options.output, std::ios::out | std::ios::app);
  if (!capture->output) {
    throw std::runtime_error("could not open output: " + options.output);
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);
  std::vector<RequestChannel> channels;
  channels.reserve(options.topics.size());
  for (const auto & topic : options.topics) {
    auto channel = unitree::robot::ChannelFactory::Instance()->CreateRecvChannel<Request>(
      topic,
      [capture, topic](const void * raw) {on_request(capture, topic, raw);},
      0);
    if (!channel || !channel->GetReader()) {
      throw std::runtime_error("could not create typed request reader for " + topic);
    }
    channels.push_back(std::move(channel));
  }
  std::printf(
    "CAPTURE status=LISTENING label=%s duration=%.1f topics=%zu\n",
    options.label.c_str(), options.duration_sec, channels.size());
  std::fflush(stdout);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<long long>(options.duration_sec * 1000.0));
  while (running.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  channels.clear();
  unitree::robot::ChannelFactory::Instance()->Release();
  std::printf("CAPTURE samples=%llu\n", capture->samples.load());
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
#if R1_APP_COMMAND_OBSERVER_HAS_SDK
    return observe(options);
#else
    (void)options;
    std::fprintf(stderr, "CAPTURE status=UNAVAILABLE detail=unitree_sdk_missing\n");
    return 20;
#endif
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "CAPTURE status=ERROR detail=%s\n", exception.what());
    return 21;
  }
}
