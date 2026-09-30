// Copyright 2026
//
// Bounded read-only Unitree R1 locomotion-state diagnostic.
//
// This executable exposes only the R1 locomotion state queries. Constructing
// the vendor Client performs its mandatory internal Noop API 2 handshake
// (which the SDK may retry). The probe also reads the service API version and
// sends API 7001; an operator may explicitly request API 7002 as well. No
// locomotion command API or command publisher is available in this executable.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>

#if R1_LOCO_STATE_PROBE_HAS_SDK
#include <dds/dds.h>
#include <dlfcn.h>
#include <link.h>
#include <net/if.h>
#include <rmw/rmw.h>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/client/client.hpp>
#include <unitree/robot/go2/public/jsonize_type.hpp>
#include <unitree/robot/internal/internal_api.hpp>
#include <unitree/robot/r1/loco/r1_loco_api.hpp>
#endif

namespace
{

constexpr int kExitOk = 0;
constexpr int kExitFsmIdFailed = 10;
constexpr int kExitFsmModeFailed = 11;
constexpr int kExitUnavailable = 20;
constexpr int kExitRuntimeError = 21;
constexpr int kExitUsage = 64;

struct Options
{
  std::string interface;
  double timeout_sec{2.0};
  bool get_fsm_mode{false};
};

void usage(const char * executable)
{
  std::printf(
    "Usage: %s --interface NAME [--timeout-sec SEC] [--get-fsm-mode]\n"
    "\n"
    "Read-only Unitree R1 locomotion-state diagnostic. SDK client "
    "construction performs its internal Noop API 2 handshake and may retry "
    "it. The probe reads the server API version, then sends one explicit API "
    "7001 query. --get-fsm-mode "
    "adds one API 7002 query after a successful API 7001 response. No "
    "state-changing locomotion API (7101, 7105, or 7107), locomotion command, "
    "or command publisher is available in this executable.\n"
    "\n"
    "Options:\n"
    "  --interface NAME   Robot-facing Ethernet interface\n"
    "  --timeout-sec SEC  Per-request timeout, 0.1..10.0 (default 2.0)\n"
    "  --get-fsm-mode     Also query the read-only FSM mode value\n"
    "  -h, --help         Show this help without initializing SDK/DDS\n"
    "\n"
    "Exit status: 0=query success; 10=explicit API 7001 failed; "
    "11=API 7002 failed; 20=SDK unavailable; 21=runtime/ABI failure; "
    "64=usage error.\n",
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
      std::exit(kExitOk);
    }
    if (argument == "--get-fsm-mode") {
      options.get_fsm_mode = true;
      continue;
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const char * const value = argv[++index];
    if (argument == "--interface") {
      options.interface = value;
    } else if (argument == "--timeout-sec") {
      options.timeout_sec = parse_double(value, "--timeout-sec");
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
  if (options.timeout_sec < 0.1 || options.timeout_sec > 10.0) {
    throw std::invalid_argument("--timeout-sec must be within 0.1..10.0");
  }
  return options;
}

#if R1_LOCO_STATE_PROBE_HAS_SDK

constexpr char kRequiredRmwImplementation[] = "rmw_fastrtps_cpp";

struct LoadedDdsLibraries
{
  std::filesystem::path ddsc;
  std::filesystem::path ddscxx;
};

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

LoadedDdsLibraries loaded_dds_libraries_or_throw()
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
  void * const ddscxx_handle = dlopen("libddscxx.so.0", RTLD_LAZY | RTLD_NOLOAD);
  if (ddscxx_handle == nullptr) {
    const char * const loader_error_text = dlerror();
    const std::string loader_error =
      loader_error_text == nullptr ? "unknown loader error" : loader_error_text;
    throw std::runtime_error(
            "DDS ABI guard cannot locate loaded libddscxx.so.0: " + loader_error);
  }

  link_map * ddscxx_link_map = nullptr;
  (void)dlerror();
  const int dlinfo_result = dlinfo(ddscxx_handle, RTLD_DI_LINKMAP, &ddscxx_link_map);
  std::string ddscxx_loaded_path;
  if (dlinfo_result == 0 && ddscxx_link_map != nullptr &&
    ddscxx_link_map->l_name != nullptr)
  {
    ddscxx_loaded_path = ddscxx_link_map->l_name;
  }
  const char * const dlinfo_error_text = dlinfo_result == 0 ? nullptr : dlerror();
  const std::string dlinfo_error =
    dlinfo_error_text == nullptr ? "" : dlinfo_error_text;
  dlclose(ddscxx_handle);

  if (dlinfo_result != 0 || ddscxx_loaded_path.empty()) {
    throw std::runtime_error(
            "DDS ABI guard cannot resolve loaded libddscxx.so.0 link map" +
            std::string(dlinfo_error.empty() ? "" : ": ") + dlinfo_error);
  }

  return {
    ddsc,
    canonical_path_or_throw(ddscxx_loaded_path, "loaded libddscxx.so.0 path")};
}

void verify_preinit_runtime_abi()
{
  const char * const configured_rmw = std::getenv("RMW_IMPLEMENTATION");
  if (configured_rmw == nullptr ||
    std::strcmp(configured_rmw, kRequiredRmwImplementation) != 0)
  {
    throw std::runtime_error(
            "DDS ABI guard requires explicit RMW_IMPLEMENTATION=" +
            std::string(kRequiredRmwImplementation) + "; got " +
            (configured_rmw == nullptr ? std::string("<unset>") :
            std::string(configured_rmw)));
  }

  const std::filesystem::path expected_directory = canonical_path_or_throw(
    R1_LOCO_STATE_PROBE_VENDOR_DDS_DIR, "compiled vendor DDS directory");
  const LoadedDdsLibraries loaded = loaded_dds_libraries_or_throw();
  if (loaded.ddsc.parent_path() != expected_directory ||
    loaded.ddscxx.parent_path() != expected_directory)
  {
    throw std::runtime_error(
            "DDS ABI guard rejected loaded libraries: expected directory=" +
            expected_directory.string() + " libddsc=" + loaded.ddsc.string() +
            " libddscxx=" + loaded.ddscxx.string());
  }

  const char * const active_rmw = rmw_get_implementation_identifier();
  if (active_rmw == nullptr || std::strcmp(active_rmw, kRequiredRmwImplementation) != 0) {
    throw std::runtime_error(
            "DDS ABI guard requires active RMW " +
            std::string(kRequiredRmwImplementation) + "; got " +
            (active_rmw == nullptr ? std::string("<null>") : std::string(active_rmw)));
  }
}

// The vendor Client constructor necessarily performs the internal API 2 Noop
// handshake. The official R1 client also registers command IDs. This narrower
// client registers only the two state-query IDs and exposes no command method.
class ReadOnlyLocoStateClient final : public unitree::robot::Client
{
public:
  explicit ReadOnlyLocoStateClient(const bool allow_fsm_mode_query)
  : Client(unitree::robot::r1::LOCO_SERVICE_NAME, false),
    allow_fsm_mode_query_(allow_fsm_mode_query)
  {
  }

  void Init() override
  {
    SetApiVersion(unitree::robot::r1::LOCO_API_VERSION);
    RegistApi(unitree::robot::r1::ROBOT_API_ID_LOCO_GET_FSM_ID);
    if (allow_fsm_mode_query_) {
      RegistApi(unitree::robot::r1::ROBOT_API_ID_LOCO_GET_FSM_MODE);
    }
  }

  int32_t get_fsm_id(int & fsm_id)
  {
    std::string request;
    std::string response;
    const int32_t status = Call(
      unitree::robot::r1::ROBOT_API_ID_LOCO_GET_FSM_ID, request, response);
    if (status != 0) {
      return status;
    }

    unitree::robot::go2::JsonizeDataInt decoded;
    unitree::common::FromJsonString(response, decoded);
    fsm_id = decoded.data;
    return status;
  }

  int32_t get_fsm_mode(int & fsm_mode)
  {
    std::string request;
    std::string response;
    const int32_t status = Call(
      unitree::robot::r1::ROBOT_API_ID_LOCO_GET_FSM_MODE, request, response);
    if (status != 0) {
      return status;
    }

    unitree::robot::go2::JsonizeDataInt decoded;
    unitree::common::FromJsonString(response, decoded);
    fsm_mode = decoded.data;
    return status;
  }

private:
  bool allow_fsm_mode_query_{false};
};

static_assert(
  unitree::robot::ROBOT_API_ID_INTERNAL_API_NOOP == 2,
  "SDK Client construction must retain the documented internal Noop API id");
static_assert(
  unitree::robot::r1::ROBOT_API_ID_LOCO_GET_FSM_ID == 7001,
  "R1 locomotion GetFsmId must remain API 7001");
static_assert(
  unitree::robot::r1::ROBOT_API_ID_LOCO_GET_FSM_MODE == 7002,
  "R1 locomotion GetFsmMode must remain API 7002");

int run_probe(const Options & options)
{
  if (if_nametoindex(options.interface.c_str()) == 0) {
    throw std::runtime_error("network interface does not exist: " + options.interface);
  }

  verify_preinit_runtime_abi();
  unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);

  ReadOnlyLocoStateClient client(options.get_fsm_mode);
  client.SetTimeout(static_cast<float>(options.timeout_sec));
  client.Init();

  const std::string local_api_version = client.GetApiVersion();
  const std::string server_api_version = client.GetServerApiVersion();
  std::printf(
    "R1_LOCO_API_VERSION local=%s server=%s match=%s\n",
    local_api_version.c_str(), server_api_version.c_str(),
    local_api_version == server_api_version ? "yes" : "no");

  int fsm_id = -1;
  const int32_t id_status = client.get_fsm_id(fsm_id);
  std::printf(
    "R1_LOCO_FSM_ID status=%s api=7001 code=%d fsm_id=%d\n",
    id_status == 0 ? "OK" : "RPC_ERROR", id_status, fsm_id);
  if (id_status != 0) {
    return kExitFsmIdFailed;
  }

  if (!options.get_fsm_mode) {
    return kExitOk;
  }

  int fsm_mode = -1;
  const int32_t mode_status = client.get_fsm_mode(fsm_mode);
  std::printf(
    "R1_LOCO_FSM_MODE status=%s api=7002 code=%d fsm_mode=%d\n",
    mode_status == 0 ? "OK" : "RPC_ERROR", mode_status, fsm_mode);
  return mode_status == 0 ? kExitOk : kExitFsmModeFailed;
}

#endif

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
#if R1_LOCO_STATE_PROBE_HAS_SDK
    return run_probe(options);
#else
    (void)options;
    std::fprintf(
      stderr, "R1_LOCO_FSM_ID status=UNAVAILABLE detail=SDK_headers_or_library_missing\n");
    return kExitUnavailable;
#endif
  } catch (const std::invalid_argument & exception) {
    std::fprintf(stderr, "R1_LOCO_FSM_ID status=USAGE_ERROR detail=%s\n", exception.what());
    return kExitUsage;
  } catch (const std::exception & exception) {
    std::fprintf(
      stderr, "R1_LOCO_FSM_ID status=RUNTIME_ERROR detail=%s\n", exception.what());
    return kExitRuntimeError;
  } catch (...) {
    std::fprintf(stderr, "R1_LOCO_FSM_ID status=RUNTIME_ERROR detail=unknown\n");
    return kExitRuntimeError;
  }
}
