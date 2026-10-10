// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scan_planner_msgs:msg/TaskAuthorization.idl
// generated code does not contain a copyright notice

#ifndef SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__STRUCT_H_
#define SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/TaskAuthorization in the package scan_planner_msgs.
/**
  * 任务授权：取消后必须由规划端**显式重新授权**才允许执行。
  *
  * 为什么需要 task_id 而不是只有一个 bool：
  *   取消时执行端会立即本地锁止（不等规划端），但可能有一条**在取消之前发布、
  *   延迟到达**的授权消息。仅凭 bool 无法区分它和新任务的授权，会重新放行旧任务。
  *   task_id 由规划端每接受一个新任务自增，执行端只接受 task_id 大于
  *   "已撤销到的编号"的授权，从而不会因延迟消息解锁。
 */
typedef struct scan_planner_msgs__msg__TaskAuthorization
{
  std_msgs__msg__Header header;
  /// true=授权执行（新任务被接受）；false=撤销
  bool active;
  /// 规划端单调自增的任务编号；撤销时携带当前编号
  uint32_t task_id;
} scan_planner_msgs__msg__TaskAuthorization;

// Struct for a sequence of scan_planner_msgs__msg__TaskAuthorization.
typedef struct scan_planner_msgs__msg__TaskAuthorization__Sequence
{
  scan_planner_msgs__msg__TaskAuthorization * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scan_planner_msgs__msg__TaskAuthorization__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCAN_PLANNER_MSGS__MSG__DETAIL__TASK_AUTHORIZATION__STRUCT_H_
