#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>

namespace flyco {

// Shared algorithm objects declare some of the same node parameters. Centralize
// that policy while keeping node ownership and parameter names explicit.
template<class T>
bool get_parameter(const rclcpp::Node::SharedPtr& node,
                   const std::string& name, T& value) {
  if (!node->has_parameter(name)) return false;
  const auto parameter = node->get_parameter(name);
  if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
    value = static_cast<T>(parameter.as_int());
  } else if constexpr (std::is_floating_point_v<T>) {
    value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
      ? static_cast<T>(parameter.as_int()) : static_cast<T>(parameter.as_double());
  } else {
    value = parameter.get_value<T>();
  }
  return true;
}

template<class T>
T parameter_or(const rclcpp::Node::SharedPtr& node,
               const std::string& name, const T& fallback) {
  if (!node->has_parameter(name)) {
    if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
      node->declare_parameter<int64_t>(name, static_cast<int64_t>(fallback));
    } else if constexpr (std::is_floating_point_v<T>) {
      node->declare_parameter<double>(name, static_cast<double>(fallback));
    } else {
      node->declare_parameter<T>(name, fallback);
    }
  }
  T value = fallback;
  get_parameter(node, name, value);
  return value;
}

template<class T>
void set_parameter(const rclcpp::Node::SharedPtr& node,
                   const std::string& name, const T& value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, value);
  }
  const auto result = node->set_parameter(rclcpp::Parameter(name, value));
  if (!result.successful) throw std::runtime_error(result.reason);
}

}  // namespace flyco
