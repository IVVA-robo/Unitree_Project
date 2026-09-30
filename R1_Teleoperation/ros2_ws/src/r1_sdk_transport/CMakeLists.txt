cmake_minimum_required(VERSION 3.8)
project(r1_sdk_transport)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Threads REQUIRED)

# The SDK is intentionally an optional build-time dependency.  This keeps the
# diagnostics node usable in CI and on a development machine without the
# vendor SDK, while enabling the real read-only LowState subscriber on the
# configured workstation.  No SDK writer header is included by this package.
set(UNITREE_SDK_ROOT
  "/home/unitree/Unitree_Project/Legacy_Robotics/robotics/unitree_sdk/unitree_sdk2"
  CACHE PATH "Root of the Unitree SDK2 checkout")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(UNITREE_SDK_ARCH "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(UNITREE_SDK_ARCH "aarch64")
else()
  set(UNITREE_SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(UNITREE_SDK_INCLUDE "${UNITREE_SDK_ROOT}/include")
set(UNITREE_SDK_THIRDPARTY_INCLUDE "${UNITREE_SDK_ROOT}/thirdparty/include")
set(UNITREE_SDK_LIB "${UNITREE_SDK_ROOT}/lib/${UNITREE_SDK_ARCH}/libunitree_sdk2.a")
set(UNITREE_SDK_LIB_DIR "${UNITREE_SDK_ROOT}/thirdparty/lib/${UNITREE_SDK_ARCH}")

set(R1_SDK_TRANSPORT_HAS_SDK OFF)
if(EXISTS "${UNITREE_SDK_INCLUDE}/unitree/idl/hg/LowState_.hpp"
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/idl/hg/LowCmd_.hpp"
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/idl/ros2/String_.hpp"
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/robot/channel/channel_factory.hpp"
   AND EXISTS "${UNITREE_SDK_LIB}"
   AND EXISTS "${UNITREE_SDK_LIB_DIR}/libddsc.so"
   AND EXISTS "${UNITREE_SDK_LIB_DIR}/libddscxx.so")
  set(R1_SDK_TRANSPORT_HAS_SDK ON)
  add_library(unitree_sdk2_vendor STATIC IMPORTED GLOBAL)
  set_target_properties(unitree_sdk2_vendor PROPERTIES
    IMPORTED_LOCATION "${UNITREE_SDK_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES
      "${UNITREE_SDK_INCLUDE};${UNITREE_SDK_THIRDPARTY_INCLUDE};${UNITREE_SDK_THIRDPARTY_INCLUDE}/ddscxx"
    INTERFACE_LINK_DIRECTORIES "${UNITREE_SDK_LIB_DIR}"
    INTERFACE_LINK_LIBRARIES "ddsc;ddscxx;fmt;Threads::Threads")
  message(STATUS "R1 SDK read-only transport enabled from ${UNITREE_SDK_ROOT}")
else()
  message(WARNING
    "Unitree SDK2 was not found under ${UNITREE_SDK_ROOT}; "
    "building an offline diagnostics-only node")
endif()

# Some SDK2 revisions carry the generic LowState headers but not the B2
# MotionSwitcher API.  Keep the rest of the read-only package buildable and
# make this probe report SDK-unavailable rather than failing compilation.
set(R1_MOTION_SWITCHER_PROBE_HAS_SDK OFF)
if(R1_SDK_TRANSPORT_HAS_SDK
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/robot/b2/motion_switcher/motion_switcher_api.hpp"
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/robot/client/client.hpp"
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/robot/internal/internal_api.hpp")
  set(R1_MOTION_SWITCHER_PROBE_HAS_SDK ON)
  message(STATUS "R1 MotionSwitcher read-only probe enabled")
else()
  message(WARNING
    "Unitree SDK2 MotionSwitcher headers were not found; "
    "building an SDK-unavailable MotionSwitcher probe")
endif()

# The R1 locomotion header also exposes state-changing calls, so the diagnostic
# uses a deliberately narrower Client subclass and registers only the two
# read-only state API IDs (7001 and optional 7002).
set(R1_LOCO_STATE_PROBE_HAS_SDK OFF)
if(R1_SDK_TRANSPORT_HAS_SDK
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/robot/r1/loco/r1_loco_api.hpp"
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/robot/go2/public/jsonize_type.hpp"
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/robot/client/client.hpp"
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/robot/internal/internal_api.hpp")
  set(R1_LOCO_STATE_PROBE_HAS_SDK ON)
  message(STATUS "R1 locomotion-state read-only probe enabled")
else()
  message(WARNING
    "Unitree SDK2 R1 locomotion-state headers were not found; "
    "building an SDK-unavailable locomotion-state probe")
endif()

set(R1_WIRELESS_CONTROLLER_OBSERVER_HAS_SDK OFF)
if(R1_SDK_TRANSPORT_HAS_SDK
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/idl/go2/WirelessController_.hpp")
  set(R1_WIRELESS_CONTROLLER_OBSERVER_HAS_SDK ON)
  message(STATUS "R1 wireless-controller read-only observer enabled")
else()
  message(WARNING
    "Unitree SDK2 WirelessController header was not found; "
    "building an SDK-unavailable wireless-controller observer")
endif()

set(R1_WIRELESS_CONTROLLER_PROBE_HAS_SDK
  ${R1_WIRELESS_CONTROLLER_OBSERVER_HAS_SDK})

set(R1_BATTERY_OBSERVER_HAS_SDK OFF)
if(R1_SDK_TRANSPORT_HAS_SDK
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/idl/hg/BmsState_.hpp")
  set(R1_BATTERY_OBSERVER_HAS_SDK ON)
  message(STATUS "R1 battery read-only observer enabled")
endif()

add_executable(r1_sdk_transport src/r1_sdk_transport.cpp)
target_compile_definitions(r1_sdk_transport PRIVATE
  R1_SDK_TRANSPORT_HAS_SDK=$<BOOL:${R1_SDK_TRANSPORT_HAS_SDK}>
  R1_SDK_TRANSPORT_VENDOR_DDS_DIR="${UNITREE_SDK_LIB_DIR}")
ament_target_dependencies(r1_sdk_transport
  geometry_msgs rclcpp rmw sensor_msgs std_msgs trajectory_msgs)
if(R1_SDK_TRANSPORT_HAS_SDK)
  target_link_libraries(r1_sdk_transport unitree_sdk2_vendor ${CMAKE_DL_LIBS})
  set_target_properties(r1_sdk_transport PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

# Separate read-only command-traffic observer.  Keeping this in its own source
# preserves r1_sdk_transport.cpp's stronger contract: the LowState transport
# itself never includes or constructs LowCmd, even as a subscriber.
add_executable(r1_arm_sdk_traffic_observer
  src/r1_arm_sdk_traffic_observer.cpp)
target_compile_definitions(r1_arm_sdk_traffic_observer PRIVATE
  R1_ARM_SDK_OBSERVER_HAS_SDK=$<BOOL:${R1_SDK_TRANSPORT_HAS_SDK}>
  R1_ARM_SDK_OBSERVER_VENDOR_DDS_DIR="${UNITREE_SDK_LIB_DIR}")
if(R1_SDK_TRANSPORT_HAS_SDK)
  target_link_libraries(r1_arm_sdk_traffic_observer
    unitree_sdk2_vendor ${CMAKE_DL_LIBS})
  set_target_properties(r1_arm_sdk_traffic_observer PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

# Bounded read-only MotionSwitcher query.  This target intentionally exposes
# only API 1001 (and opt-in API 1005); it contains no mode-selection path.
add_executable(r1_motion_switcher_probe
  src/r1_motion_switcher_probe.cpp)
target_compile_definitions(r1_motion_switcher_probe PRIVATE
  R1_MOTION_SWITCHER_PROBE_HAS_SDK=$<BOOL:${R1_MOTION_SWITCHER_PROBE_HAS_SDK}>
  R1_MOTION_SWITCHER_PROBE_VENDOR_DDS_DIR="${UNITREE_SDK_LIB_DIR}")
ament_target_dependencies(r1_motion_switcher_probe
  rmw rmw_fastrtps_cpp)
if(R1_MOTION_SWITCHER_PROBE_HAS_SDK)
  target_link_libraries(r1_motion_switcher_probe
    unitree_sdk2_vendor ${CMAKE_DL_LIBS})
  set_target_properties(r1_motion_switcher_probe PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

# Bounded read-only query of the R1 locomotion service. This target exposes
# only API 7001 (and opt-in API 7002); it contains no FSM/velocity command path.
add_executable(r1_loco_state_probe
  src/r1_loco_state_probe.cpp)
target_compile_definitions(r1_loco_state_probe PRIVATE
  R1_LOCO_STATE_PROBE_HAS_SDK=$<BOOL:${R1_LOCO_STATE_PROBE_HAS_SDK}>
  R1_LOCO_STATE_PROBE_VENDOR_DDS_DIR="${UNITREE_SDK_LIB_DIR}")
ament_target_dependencies(r1_loco_state_probe
  rmw rmw_fastrtps_cpp)
if(R1_LOCO_STATE_PROBE_HAS_SDK)
  target_link_libraries(r1_loco_state_probe
    unitree_sdk2_vendor ${CMAKE_DL_LIBS})
  set_target_properties(r1_loco_state_probe PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

# Bounded typed subscriber for the official controller channel.  This target
# has no publisher or command API and is safe to run while diagnosing input
# ownership and routing.
add_executable(r1_wireless_controller_observer
  src/r1_wireless_controller_observer.cpp)
target_compile_definitions(r1_wireless_controller_observer PRIVATE
  R1_WIRELESS_CONTROLLER_OBSERVER_HAS_SDK=$<BOOL:${R1_WIRELESS_CONTROLLER_OBSERVER_HAS_SDK}>
  R1_WIRELESS_CONTROLLER_OBSERVER_VENDOR_DDS_DIR="${UNITREE_SDK_LIB_DIR}")
if(R1_WIRELESS_CONTROLLER_OBSERVER_HAS_SDK)
  target_link_libraries(r1_wireless_controller_observer
    unitree_sdk2_vendor ${CMAKE_DL_LIBS})
  set_target_properties(r1_wireless_controller_observer PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

# Isolated commissioning publisher.  Its source hard-codes a single bounded
# forward-stick pulse and zero frames; a separate acknowledgement gates the
# live pulse at runtime.
add_executable(r1_wireless_controller_probe
  src/r1_wireless_controller_probe.cpp)
target_compile_definitions(r1_wireless_controller_probe PRIVATE
  R1_WIRELESS_CONTROLLER_PROBE_HAS_SDK=$<BOOL:${R1_WIRELESS_CONTROLLER_PROBE_HAS_SDK}>
  R1_WIRELESS_CONTROLLER_PROBE_VENDOR_DDS_DIR="${UNITREE_SDK_LIB_DIR}")
if(R1_WIRELESS_CONTROLLER_PROBE_HAS_SDK)
  target_link_libraries(r1_wireless_controller_probe
    unitree_sdk2_vendor ${CMAKE_DL_LIBS})
  set_target_properties(r1_wireless_controller_probe PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

# Always-on read-only SOC monitor used by the operator panel. It subscribes to
# typed BmsState topics only and contains no writer, RPC client, or command API.
add_executable(r1_battery_observer src/r1_battery_observer.cpp)
target_compile_definitions(r1_battery_observer PRIVATE
  R1_BATTERY_OBSERVER_HAS_SDK=$<BOOL:${R1_BATTERY_OBSERVER_HAS_SDK}>)
if(R1_BATTERY_OBSERVER_HAS_SDK)
  target_link_libraries(r1_battery_observer
    unitree_sdk2_vendor ${CMAKE_DL_LIBS})
  set_target_properties(r1_battery_observer PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

# Read-only recorder for commands emitted by Unitree Explore. It subscribes to
# typed Request topics only and writes JSONL; there is deliberately no writer
# or RPC client in this target.
add_executable(r1_app_command_observer src/r1_app_command_observer.cpp)
target_compile_definitions(r1_app_command_observer PRIVATE
  R1_APP_COMMAND_OBSERVER_HAS_SDK=$<BOOL:${R1_BATTERY_OBSERVER_HAS_SDK}>)
if(R1_BATTERY_OBSERVER_HAS_SDK)
  target_link_libraries(r1_app_command_observer
    unitree_sdk2_vendor ${CMAKE_DL_LIBS})
  set_target_properties(r1_app_command_observer PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

configure_file(
  launch/r1_sdk_transport.launch.py
  ${CMAKE_CURRENT_BINARY_DIR}/r1_sdk_transport.launch.py
  @ONLY)

install(TARGETS r1_sdk_transport r1_arm_sdk_traffic_observer
  r1_motion_switcher_probe r1_loco_state_probe
  r1_wireless_controller_observer
  r1_wireless_controller_probe
  r1_battery_observer
  r1_app_command_observer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/r1_sdk_transport.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_r1_sdk_transport_package_contract
    test/test_r1_sdk_transport_package_contract.py)
  ament_add_pytest_test(test_traffic_gate_entrypoints
    test/test_traffic_gate_entrypoints.py)
  ament_add_pytest_test(test_motion_switcher_probe_contract
    test/test_motion_switcher_probe_contract.py)
  ament_add_pytest_test(test_loco_state_probe_contract
    test/test_loco_state_probe_contract.py)
  ament_add_pytest_test(test_wireless_controller_observer_contract
    test/test_wireless_controller_observer_contract.py)
    ament_add_pytest_test(test_wireless_controller_probe_contract
      test/test_wireless_controller_probe_contract.py)
    ament_add_pytest_test(test_panel_observers_contract
      test/test_panel_observers_contract.py)
  if(R1_SDK_TRANSPORT_HAS_SDK)
    ament_add_pytest_test(test_runtime_abi_guard
      test/test_runtime_abi_guard.py
      ENV
        "R1_SDK_TRANSPORT_BINARY=$<TARGET_FILE:r1_sdk_transport>"
        "R1_ARM_SDK_OBSERVER_BINARY=$<TARGET_FILE:r1_arm_sdk_traffic_observer>"
        "R1_MOTION_SWITCHER_PROBE_BINARY=$<TARGET_FILE:r1_motion_switcher_probe>"
        "R1_LOCO_STATE_PROBE_BINARY=$<TARGET_FILE:r1_loco_state_probe>"
        "R1_WIRELESS_CONTROLLER_OBSERVER_BINARY=$<TARGET_FILE:r1_wireless_controller_observer>"
        "R1_WIRELESS_CONTROLLER_PROBE_BINARY=$<TARGET_FILE:r1_wireless_controller_probe>"
        "R1_SDK_TRANSPORT_VENDOR_DDS_DIR=${UNITREE_SDK_LIB_DIR}"
      TIMEOUT 30)
  endif()
endif()

ament_package()
