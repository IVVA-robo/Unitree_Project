// Copyright 2026
//
// Bounded read-only Unitree MotionSwitcher diagnostic.
//
// This executable has one purpose: query the currently reported motion form
// and name.  It never changes ownership, operating mode, silence state, or
// motor state.  Constructing the vendor Client performs its mandatory internal
// Noop API 2 handshake (which the SDK may retry).  That is not a
// MotionSwitcher mode operation.  The only explicit MotionSwitcher query by
// default is API 1001; an operator may explicitly request API 1005 as well.

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

#if R1_MOTION_SWITCHER_PROBE_HAS_SDK
#include <dds/dds.h>
#include <dlfcn.h>
#include <link.h>
#include <net/if.h>
#include <rmw/rmw.h>
#include <unitree/robot/b2/motion_switcher/motion_switcher_api.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/client/client.hpp>
#include <unitree/robot/internal/internal_api.hpp>
#endif

namespace
{

constexpr int kExitOk = 0;
constexpr int kExitCheckFailed = 10;
constexpr int kExitSilentFailed = 11;
constexpr int kExitUnavailable = 20;
constexpr int kExitRuntimeError = 21;
constexpr int kExitUsage = 64;

struct Options
{
  std::string interface;
  double timeout_sec{2.0};
  bool get_silent{false};
};

void usage(const char * executable)
{
  std::printf(
    "Usage: %s --interface NAME [--timeout-sec SEC] [--get-silent]\n"
    "\n"
    "Read-only Unitree MotionSwitcher diagnostic. SDK client construction "
    "performs its internal Noop API 2 handshake and may retry it. The probe "
    "then sends one explicit API 1001 query. --get-silent adds one API 1005 "
    "query after a successful API 1001 response. No state-changing "
    "MotionSwitcher API (1002, 1003, or 1004) or command publisher is "
    "available in this executable.\n"
    "\n"
    "Options:\n"
    "  --interface NAME   Robot-facing Ethernet interface\n"
    "  --timeout-sec SEC  Per-request timeout, 0.1..10.0 (default 2.0)\n"
    "  --get-silent       Also query the read-only silent-state value\n"
    "  -h, --help         Show this help without initializing SDK/DDS\n"
    "\n"
    "Exit status: 0=query success; 10=explicit API 1001 failed; "
    "11=API 1005 failed; 20=SDK unavailable; 21=runtime/ABI failure; "
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
    if (argument == "--get-silent") {
      options.get_silent = true;
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

std::string printable_text(const std::string & text)
{
  std::string result;
  result.reserve(std::min<std::size_t>(text.size(), 256));
  for (const unsigned char character : text) {
    if (result.size() == 256) {
      result += "...";
      break;
    }
    result += std::isprint(character) ? static_cast<char>(character) : '?';
  }
  return result;
}

#if R1_MOTION_SWITCHER_PROBE_HAS_SDK

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
    R1_MOTION_SWITCHER_PROBE_VENDOR_DDS_DIR, "compiled vendor DDS directory");
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
// handshake.  The official MotionSwitcher client otherwise registers all five
// mode APIs.  This narrower client registers only the two query IDs and
// exposes no state-changing MotionSwitcher request path.
class ReadOnlyMotionSwitcherClient final : public unitree::robot::Client
{
public:
  explicit ReadOnlyMotionSwitcherClient(const bool allow_silent_query)
  : Client(unitree::robot::b2::MOTION_SWITCHER_SERVICE_NAME),
    allow_silent_query_(allow_silent_query)
  {
  }

  void Init() override
  {
    SetApiVersion(unitree::robot::b2::MOTION_SWITCHER_API_VERSION);
    RegistApi(unitree::robot::b2::MOTION_SWITCHER_API_ID_CHECK_MODE);
    if (allow_silent_query_) {
      RegistApi(unitree::robot::b2::MOTION_SWITCHER_API_ID_GET_SILENT);
    }
  }

  int32_t check_mode(std::string & form, std::string & name)
  {
    std::string request;
    std::string response;
    const int32_t status = Call(
      unitree::robot::b2::MOTION_SWITCHER_API_ID_CHECK_MODE, request, response);
    if (status != 0) {
      return status;
    }

    unitree::robot::b2::JsonizeModeName decoded;
    unitree::common::FromJsonString(response, decoded);
    form = decoded.form;
    name = decoded.name;
    return status;
  }

  int32_t get_silent(bool & silent)
  {
    std::string request;
    std::string response;
    const int32_t status = Call(
      unitree::robot::b2::MOTION_SWITCHER_API_ID_GET_SILENT, request, response);
    if (status != 0) {
      return status;
    }

    unitree::robot::b2::JsonizeSilent decoded;
    unitree::common::FromJsonString(response, decoded);
    silent = decoded.silent;
    return status;
  }

private:
  bool allow_silent_query_{false};
};

static_assert(
  unitree::robot::ROBOT_API_ID_INTERNAL_API_NOOP == 2,
  "SDK Client construction must retain the documented internal Noop API id");
static_assert(
  unitree::robot::b2::MOTION_SWITCHER_API_ID_CHECK_MODE == 1001,
  "MotionSwitcher CheckMode must remain API 1001");
static_assert(
  unitree::robot::b2::MOTION_SWITCHER_API_ID_GET_SILENT == 1005,
  "MotionSwitcher GetSilent must remain API 1005");

int run_probe(const Options & options)
{
  if (if_nametoindex(options.interface.c_str()) == 0) {
    throw std::runtime_error("network interface does not exist: " + options.interface);
  }

  verify_preinit_runtime_abi();
  unitree::robot::ChannelFactory::Instance()->Init(0, options.interface);

  ReadOnlyMotionSwitcherClient client(options.get_silent);
  client.SetTimeout(static_cast<float>(options.timeout_sec));
  client.Init();

  std::string form;
  std::string name;
  const int32_t check_status = client.check_mode(form, name);
  std::printf(
    "MOTION_SWITCHER_CHECK status=%s api=1001 code=%d form=%s name=%s\n",
    check_status == 0 ? "OK" : "RPC_ERROR", check_status,
    printable_text(form).c_str(), printable_text(name).c_str());
  if (check_status != 0) {
    return kExitCheckFailed;
  }

  if (!options.get_silent) {
    return kExitOk;
  }

  bool silent = false;
  const int32_t silent_status = client.get_silent(silent);
  std::printf(
    "MOTION_SWITCHER_SILENT status=%s api=1005 code=%d silent=%s\n",
    silent_status == 0 ? "OK" : "RPC_ERROR", silent_status,
    silent ? "true" : "false");
  return silent_status == 0 ? kExitOk : kExitSilentFailed;
}

#endif

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
#if R1_MOTION_SWITCHER_PROBE_HAS_SDK
    return run_probe(options);
#else
    (void)options;
    std::fprintf(
      stderr,
      "MOTION_SWITCHER_CHECK status=UNAVAILABLE detail=SDK headers_or_library_missing\n");
    return kExitUnavailable;
#endif
  } catch (const std::exception & exception) {
    std::fprintf(
      stderr, "MOTION_SWITCHER_CHECK status=RUNTIME_ERROR detail=%s\n", exception.what());
    return kExitRuntimeError;
  } catch (...) {
    std::fprintf(
      stderr, "MOTION_SWITCHER_CHECK status=RUNTIME_ERROR detail=unknown\n");
    return kExitRuntimeError;
  }
}
