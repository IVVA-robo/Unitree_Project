#include "r1_live_writer/transport.hpp"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

int main(int argc, char ** argv)
{
  constexpr char kRequiredRmw[] = "rmw_fastrtps_cpp";
  const char * const requested_rmw = std::getenv("RMW_IMPLEMENTATION");
  if (requested_rmw == nullptr || std::string(requested_rmw) != kRequiredRmw) {
    std::fprintf(
      stderr, "ABI probe requires RMW_IMPLEMENTATION=%s before rclcpp init\n",
      kRequiredRmw);
    return 2;
  }
  rclcpp::init(argc, argv);

  int exit_code = 1;
  try {
    const auto node = std::make_shared<rclcpp::Node>("r1_sdk_abi_probe");
    const char * const active_rmw = rmw_get_implementation_identifier();
    if (active_rmw == nullptr || std::string(active_rmw) != kRequiredRmw) {
      RCLCPP_ERROR(
        node->get_logger(), "ABI probe active RMW mismatch: expected=%s actual=%s",
        kRequiredRmw, active_rmw == nullptr ? "unavailable" : active_rmw);
      rclcpp::shutdown();
      return 2;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Starting inert Unitree SDK ABI probe using interface lo");

    r1_live_writer::SdkTransport transport;
    const auto result = transport.initialize("lo", true, true);
    if (result.ok) {
      RCLCPP_INFO(node->get_logger(), "ABI probe succeeded: %s", result.detail.c_str());
      exit_code = 0;
    } else {
      RCLCPP_ERROR(node->get_logger(), "ABI probe failed: %s", result.detail.c_str());
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("r1_sdk_abi_probe"), "ABI probe exception: %s", exception.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("r1_sdk_abi_probe"), "ABI probe failed with unknown exception");
  }

  rclcpp::shutdown();
  return exit_code;
}
