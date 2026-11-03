#pragma once
#include <rclcpp/clock.hpp>

namespace flyco {
// Diagnostic throttling must remain monotonic across simulation clock resets.
inline rclcpp::Clock& steady_logging_clock() {
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}
}  // namespace flyco
