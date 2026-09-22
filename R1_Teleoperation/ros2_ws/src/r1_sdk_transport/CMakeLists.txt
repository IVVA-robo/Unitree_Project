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

configure_file(
  launch/r1_sdk_transport.launch.py
  ${CMAKE_CURRENT_BINARY_DIR}/r1_sdk_transport.launch.py
  @ONLY)

install(TARGETS r1_sdk_transport r1_arm_sdk_traffic_observer
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
  if(R1_SDK_TRANSPORT_HAS_SDK)
    ament_add_pytest_test(test_runtime_abi_guard
      test/test_runtime_abi_guard.py
      ENV
        "R1_SDK_TRANSPORT_BINARY=$<TARGET_FILE:r1_sdk_transport>"
        "R1_ARM_SDK_OBSERVER_BINARY=$<TARGET_FILE:r1_arm_sdk_traffic_observer>"
        "R1_SDK_TRANSPORT_VENDOR_DDS_DIR=${UNITREE_SDK_LIB_DIR}"
      TIMEOUT 30)
  endif()
endif()

ament_package()
