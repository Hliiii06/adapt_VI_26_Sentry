// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from scan_planner_msgs:msg/TaskAuthorization.idl
// generated code does not contain a copyright notice

#ifndef SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__TRAITS_HPP_
#define SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "scan_planner_msgs/msg/detail/task_authorization__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace scan_planner_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TaskAuthorization & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: active
  {
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << ", ";
  }

  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TaskAuthorization & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << "\n";
  }

  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TaskAuthorization & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace scan_planner_msgs

namespace rosidl_generator_traits
{

[[deprecated("use scan_planner_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const scan_planner_msgs::msg::TaskAuthorization & msg,
  std::ostream & out, size_t indentation = 0)
{
  scan_planner_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use scan_planner_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const scan_planner_msgs::msg::TaskAuthorization & msg)
{
  return scan_planner_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<scan_planner_msgs::msg::TaskAuthorization>()
{
  return "scan_planner_msgs::msg::TaskAuthorization";
}

template<>
inline const char * name<scan_planner_msgs::msg::TaskAuthorization>()
{
  return "scan_planner_msgs/msg/TaskAuthorization";
}

template<>
struct has_fixed_size<scan_planner_msgs::msg::TaskAuthorization>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<scan_planner_msgs::msg::TaskAuthorization>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<scan_planner_msgs::msg::TaskAuthorization>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__TRAITS_HPP_
