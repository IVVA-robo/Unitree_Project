from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
PROBE = (ROOT / "src" / "r1_sdk_abi_probe.cpp").read_text()
NODE = (ROOT / "src" / "r1_live_writer_node.cpp").read_text()
TRANSPORT = (ROOT / "src" / "transport.cpp").read_text()
WRAPPER = (ROOT / "scripts" / "r1-sdk-abi-probe").read_text()
CMAKE = (ROOT / "CMakeLists.txt").read_text()


def test_probe_has_one_inert_sdk_boundary():
    assert 'rclcpp::Node>("r1_sdk_abi_probe")' in PROBE
    assert 'initialize("lo", true, true)' in PROBE
    assert len(re.findall(r"\binitialize\s*\(", PROBE)) == 1

    forbidden_calls = (
        "prepare",
        "set_velocity",
        "stop_locomotion",
        "seed_head",
        "command_head",
        "hold_head",
        "release_head",
        "publish",
    )
    for method in forbidden_calls:
        assert not re.search(rf"(?:\.|->)\s*{method}\s*\(", PROBE), method

    assert "rclcpp::spin" not in PROBE
    assert "create_publisher" not in PROBE
    assert "create_subscription" not in PROBE


def test_wrapper_has_no_host_network_execution_path():
    assert "unshare --user --map-root-user --net" in WRAPPER
    assert "/proc/self/ns/net" in WRAPPER
    assert "exec 9< /proc/self/ns/net" in WRAPPER
    assert "/proc/self/fd/9" in WRAPPER
    assert 'CURRENT_NETWORK_NAMESPACE}" != "${CALLER_NETWORK_NAMESPACE' in WRAPPER
    assert "ip link set dev lo up" in WRAPPER
    assert "ip -o link show" in WRAPPER
    assert 'name != "lo"' in WRAPPER
    assert "refusing host-network fallback" in WRAPPER


def test_wrapper_pins_middleware_and_vendor_cyclonedds_pair():
    vendor = (
        "/home/unitree/Unitree_Project/Legacy_Robotics/robotics/"
        "unitree_sdk/unitree_sdk2/thirdparty/lib/x86_64"
    )
    assert vendor in WRAPPER
    assert 'RMW_IMPLEMENTATION="rmw_fastrtps_cpp"' in WRAPPER
    assert re.search(
        r'LD_LIBRARY_PATH="\$\{VENDOR_LIBRARY_DIRECTORY\}'
        r'\$\{LD_LIBRARY_PATH:\+:\$\{LD_LIBRARY_PATH\}\}"',
        WRAPPER,
    )
    assert 'verify_vendor_library "libddsc.so.0"' in WRAPPER
    assert 'verify_vendor_library "libddscxx.so.0"' in WRAPPER
    assert "ldd --" in WRAPPER
    assert "=> not found" in WRAPPER


def test_wrapper_bounds_probe_lifetime_and_fails_closed():
    assert 'PROBE_TIMEOUT_SECONDS="15"' in WRAPPER
    assert "timeout --foreground --signal=TERM --kill-after=2s" in WRAPPER
    assert "124|137)" in WRAPPER
    assert "exec unshare" in WRAPPER
    assert "sudo" not in WRAPPER


def test_probe_and_wrapper_are_installed_and_tested():
    assert "add_executable(r1_sdk_abi_probe" in CMAKE
    assert re.search(
        r"install\(TARGETS[^)]*\br1_sdk_abi_probe\b",
        CMAKE,
        flags=re.DOTALL,
    )
    assert "scripts/r1-sdk-abi-probe" in CMAKE
    assert "test_abi_probe_contract.py" in CMAKE


def test_process_and_transport_both_fail_closed_before_unitree_init():
    """Protect both middleware selection and the actual loader result."""
    assert 'RMW_IMPLEMENTATION' in NODE
    assert 'R1_PHYSICAL_SDK_SESSION' in NODE
    assert NODE.index('physical r1_live_writer requires RMW_IMPLEMENTATION=') \
        < NODE.index('rclcpp::init(argc, argv)')
    assert 'rmw_get_implementation_identifier()' in NODE
    assert 'SDK transport requires active RMW' in NODE

    guard = TRANSPORT.index('verify_vendor_dds_libraries()')
    channel = TRANSPORT.index(
        'unitree::robot::ChannelFactory::Instance()->Init', guard)
    assert guard < channel
    for required in (
        'dds_create_participant',
        'RTLD_NOLOAD',
        'RTLD_DI_LINKMAP',
        'libddsc=',
        'libddscxx=',
    ):
        assert required in TRANSPORT
