#include <rclcpp/rclcpp.hpp>
#ifndef __UAV_UTILS_H
#define __UAV_UTILS_H

#include <cstdlib>

#include <uav_utils/converters.h>
#include <uav_utils/geometry_utils.h>

namespace uav_utils {

/* judge if value belongs to [low,high] */
template <typename T, typename T2>
bool in_range(T value, const T2& low, const T2& high) {
  if (!(low < high)) { RCLCPP_FATAL(rclcpp::get_logger("flyco.uav_utils"), "Invalid range"); std::abort(); }
  return (low <= value) && (value <= high);
}

/* judge if value belongs to [-limit, limit] */
template <typename T, typename T2>
bool in_range(T value, const T2& limit) {
  if (!(limit > 0)) { RCLCPP_FATAL(rclcpp::get_logger("flyco.uav_utils"), "Invalid limit"); std::abort(); }
  return in_range(value, -limit, limit);
}

template <typename T, typename T2>
void limit_range(T& value, const T2& low, const T2& high) {
  if (!(low < high)) { RCLCPP_FATAL(rclcpp::get_logger("flyco.uav_utils"), "Invalid range"); std::abort(); }
  if (value < low) {
    value = low;
  }

  if (value > high) {
    value = high;
  }

  return;
}

template <typename T, typename T2>
void limit_range(T& value, const T2& limit) {
  if (!(limit > 0)) { RCLCPP_FATAL(rclcpp::get_logger("flyco.uav_utils"), "Invalid limit"); std::abort(); }
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
}  // end of namespace uav_utils

#endif
