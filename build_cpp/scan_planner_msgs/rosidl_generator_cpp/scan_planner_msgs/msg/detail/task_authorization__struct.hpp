// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scan_planner_msgs:msg/TaskAuthorization.idl
// generated code does not contain a copyright notice

#ifndef SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__STRUCT_HPP_
#define SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__scan_planner_msgs__msg__TaskAuthorization __attribute__((deprecated))
#else
# define DEPRECATED__scan_planner_msgs__msg__TaskAuthorization __declspec(deprecated)
#endif

namespace scan_planner_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TaskAuthorization_
{
  using Type = TaskAuthorization_<ContainerAllocator>;

  explicit TaskAuthorization_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active = false;
      this->task_id = 0ul;
    }
  }

  explicit TaskAuthorization_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active = false;
      this->task_id = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _active_type =
    bool;
  _active_type active;
  using _task_id_type =
    uint32_t;
  _task_id_type task_id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__active(
    const bool & _arg)
  {
    this->active = _arg;
    return *this;
  }
  Type & set__task_id(
    const uint32_t & _arg)
  {
    this->task_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator> *;
  using ConstRawPtr =
    const scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scan_planner_msgs__msg__TaskAuthorization
    std::shared_ptr<scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scan_planner_msgs__msg__TaskAuthorization
    std::shared_ptr<scan_planner_msgs::msg::TaskAuthorization_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TaskAuthorization_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->active != other.active) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const TaskAuthorization_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TaskAuthorization_

// alias to use template instance with default allocator
using TaskAuthorization =
  scan_planner_msgs::msg::TaskAuthorization_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace scan_planner_msgs

#endif  // SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__STRUCT_HPP_
