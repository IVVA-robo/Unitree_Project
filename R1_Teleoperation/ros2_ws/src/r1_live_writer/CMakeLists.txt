cmake_minimum_required(VERSION 3.8)
project(r1_live_writer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Threads REQUIRED)

# SDK support is optional so CI and a development laptop can still compile and
# exercise the complete safety state machine.  Finding the SDK does not open a
# DDS participant: the SDK transport is constructed only after the runtime
# authorization gate succeeds.
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

set(R1_LIVE_WRITER_HAS_SDK OFF)
if(EXISTS "${UNITREE_SDK_INCLUDE}/unitree/dds_wrapper/robots/r1/r1.h"
   AND EXISTS "${UNITREE_SDK_INCLUDE}/unitree/robot/r1/loco/r1_loco_client.hpp"
   AND EXISTS "${UNITREE_SDK_LIB}"
   AND EXISTS "${UNITREE_SDK_LIB_DIR}/libddsc.so"
   AND EXISTS "${UNITREE_SDK_LIB_DIR}/libddscxx.so")
  set(R1_LIVE_WRITER_HAS_SDK ON)
  add_library(r1_live_writer_unitree_sdk STATIC IMPORTED GLOBAL)
  set_target_properties(r1_live_writer_unitree_sdk PROPERTIES
    IMPORTED_LOCATION "${UNITREE_SDK_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES
      "${UNITREE_SDK_INCLUDE};${UNITREE_SDK_THIRDPARTY_INCLUDE};${UNITREE_SDK_THIRDPARTY_INCLUDE}/ddscxx"
    INTERFACE_LINK_DIRECTORIES "${UNITREE_SDK_LIB_DIR}"
    INTERFACE_LINK_LIBRARIES "ddsc;ddscxx;fmt;Threads::Threads")
  message(STATUS "R1 live writer SDK support compiled from ${UNITREE_SDK_ROOT}")
else()
  message(WARNING
    "Unitree SDK2 not found at ${UNITREE_SDK_ROOT}; SDK transport will fail closed")
endif()

add_library(r1_live_writer_core
  src/arm_sdk_frame.cpp
  src/safety.cpp
  src/transport.cpp)
target_include_directories(r1_live_writer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(r1_live_writer_core PUBLIC
  R1_LIVE_WRITER_HAS_SDK=$<BOOL:${R1_LIVE_WRITER_HAS_SDK}>)
if(R1_LIVE_WRITER_HAS_SDK)
  target_link_libraries(r1_live_writer_core r1_live_writer_unitree_sdk)
  set_target_properties(r1_live_writer_core PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

add_executable(r1_live_writer_node src/r1_live_writer_node.cpp)
target_include_directories(r1_live_writer_node PRIVATE include)
target_compile_definitions(r1_live_writer_node PRIVATE
  R1_LIVE_WRITER_HAS_SDK=$<BOOL:${R1_LIVE_WRITER_HAS_SDK}>)
ament_target_dependencies(r1_live_writer_node
  geometry_msgs rclcpp rmw rmw_implementation sensor_msgs std_msgs std_srvs
  trajectory_msgs)
target_link_libraries(r1_live_writer_node r1_live_writer_core Threads::Threads)
if(R1_LIVE_WRITER_HAS_SDK)
  set_target_properties(r1_live_writer_node PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

add_executable(r1_sdk_abi_probe src/r1_sdk_abi_probe.cpp)
target_include_directories(r1_sdk_abi_probe PRIVATE include)
ament_target_dependencies(r1_sdk_abi_probe rclcpp rmw rmw_implementation)
target_link_libraries(r1_sdk_abi_probe r1_live_writer_core Threads::Threads)
if(R1_LIVE_WRITER_HAS_SDK)
  set_target_properties(r1_sdk_abi_probe PROPERTIES
    BUILD_RPATH "${UNITREE_SDK_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK_LIB_DIR}")
endif()

install(TARGETS r1_live_writer_core r1_live_writer_node r1_sdk_abi_probe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/r1-sdk-abi-probe
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_safety test/test_safety.cpp)
  ament_add_gtest(test_sequential_control test/test_sequential_control.cpp)
  if(TARGET test_sequential_control)
    target_link_libraries(test_sequential_control r1_live_writer_core)
    target_include_directories(test_sequential_control PRIVATE include)
  endif()
  if(TARGET test_safety)
    target_link_libraries(test_safety r1_live_writer_core)
    target_include_directories(test_safety PRIVATE include)
  endif()
  if(R1_LIVE_WRITER_HAS_SDK)
    ament_add_gtest(test_arm_sdk_frame test/test_arm_sdk_frame.cpp)
    if(TARGET test_arm_sdk_frame)
      target_link_libraries(test_arm_sdk_frame r1_live_writer_core)
    endif()
  endif()
  ament_add_pytest_test(test_abi_probe_contract test/test_abi_probe_contract.py)
  ament_add_pytest_test(
    test_head_ownership_probe_contract
    test/test_head_ownership_probe_contract.py)
  ament_add_pytest_test(
    test_r1_live_writer_package_contract
    test/test_r1_live_writer_package_contract.py)
  ament_add_pytest_test(test_process_guards test/test_process_guards.py
    TIMEOUT 15
    ENV
      "R1_LIVE_WRITER_TEST_EXECUTABLE=$<TARGET_FILE:r1_live_writer_node>"
      "R1_UNITREE_TEST_LIBRARY_DIR=${UNITREE_SDK_LIB_DIR}")
  ament_add_pytest_test(test_staged_scripts test/test_staged_scripts.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(r1_live_writer_core)
ament_package()
