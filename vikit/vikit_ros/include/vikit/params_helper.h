/* Parameter helpers for native ROS 2. Original author: cforster. */
#pragma once
#include <algorithm>
#include <string>
#include <rclcpp/rclcpp.hpp>
namespace vk {
inline std::string parameterName(std::string name)
{
  std::replace(name.begin(), name.end(), '/', '.');
  while (!name.empty() && name.front() == '.') name.erase(name.begin());
  return name;
}
template<typename T>
T param(rclcpp::Node& node, const std::string& name, const T& value, bool = false)
{
  const auto key = parameterName(name);
  if (node.has_parameter(key)) return node.get_parameter(key).get_value<T>();
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return node.declare_parameter<T>(key, value, descriptor);
}
}  // namespace vk
