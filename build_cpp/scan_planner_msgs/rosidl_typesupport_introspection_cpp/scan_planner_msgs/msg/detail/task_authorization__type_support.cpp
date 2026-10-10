// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from scan_planner_msgs:msg/TaskAuthorization.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "scan_planner_msgs/msg/detail/task_authorization__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace scan_planner_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TaskAuthorization_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) scan_planner_msgs::msg::TaskAuthorization(_init);
}

void TaskAuthorization_fini_function(void * message_memory)
{
  auto typed_message = static_cast<scan_planner_msgs::msg::TaskAuthorization *>(message_memory);
  typed_message->~TaskAuthorization();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TaskAuthorization_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scan_planner_msgs::msg::TaskAuthorization, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "active",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scan_planner_msgs::msg::TaskAuthorization, active),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "task_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scan_planner_msgs::msg::TaskAuthorization, task_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TaskAuthorization_message_members = {
  "scan_planner_msgs::msg",  // message namespace
  "TaskAuthorization",  // message name
  3,  // number of fields
  sizeof(scan_planner_msgs::msg::TaskAuthorization),
  TaskAuthorization_message_member_array,  // message members
  TaskAuthorization_init_function,  // function to initialize message memory (memory has to be allocated)
  TaskAuthorization_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TaskAuthorization_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TaskAuthorization_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace scan_planner_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<scan_planner_msgs::msg::TaskAuthorization>()
{
  return &::scan_planner_msgs::msg::rosidl_typesupport_introspection_cpp::TaskAuthorization_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, scan_planner_msgs, msg, TaskAuthorization)() {
  return &::scan_planner_msgs::msg::rosidl_typesupport_introspection_cpp::TaskAuthorization_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
