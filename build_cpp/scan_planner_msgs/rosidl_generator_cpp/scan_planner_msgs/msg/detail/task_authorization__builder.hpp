// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scan_planner_msgs:msg/TaskAuthorization.idl
// generated code does not contain a copyright notice

#ifndef SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__BUILDER_HPP_
#define SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "scan_planner_msgs/msg/detail/task_authorization__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace scan_planner_msgs
{

namespace msg
{

namespace builder
{

class Init_TaskAuthorization_task_id
{
public:
  explicit Init_TaskAuthorization_task_id(::scan_planner_msgs::msg::TaskAuthorization & msg)
  : msg_(msg)
  {}
  ::scan_planner_msgs::msg::TaskAuthorization task_id(::scan_planner_msgs::msg::TaskAuthorization::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scan_planner_msgs::msg::TaskAuthorization msg_;
};

class Init_TaskAuthorization_active
{
public:
  explicit Init_TaskAuthorization_active(::scan_planner_msgs::msg::TaskAuthorization & msg)
  : msg_(msg)
  {}
  Init_TaskAuthorization_task_id active(::scan_planner_msgs::msg::TaskAuthorization::_active_type arg)
  {
    msg_.active = std::move(arg);
    return Init_TaskAuthorization_task_id(msg_);
  }

private:
  ::scan_planner_msgs::msg::TaskAuthorization msg_;
};

class Init_TaskAuthorization_header
{
public:
  Init_TaskAuthorization_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskAuthorization_active header(::scan_planner_msgs::msg::TaskAuthorization::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TaskAuthorization_active(msg_);
  }

private:
  ::scan_planner_msgs::msg::TaskAuthorization msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scan_planner_msgs::msg::TaskAuthorization>()
{
  return scan_planner_msgs::msg::builder::Init_TaskAuthorization_header();
}

}  // namespace scan_planner_msgs

#endif  // SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__BUILDER_HPP_
