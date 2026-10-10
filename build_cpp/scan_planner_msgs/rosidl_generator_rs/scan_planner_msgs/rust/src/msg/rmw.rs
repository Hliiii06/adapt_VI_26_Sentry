#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "scan_planner_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__scan_planner_msgs__msg__Bspline() -> *const std::ffi::c_void;
}

#[link(name = "scan_planner_msgs__rosidl_generator_c")]
extern "C" {
    fn scan_planner_msgs__msg__Bspline__init(msg: *mut Bspline) -> bool;
    fn scan_planner_msgs__msg__Bspline__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Bspline>, size: usize) -> bool;
    fn scan_planner_msgs__msg__Bspline__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Bspline>);
    fn scan_planner_msgs__msg__Bspline__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Bspline>, out_seq: *mut rosidl_runtime_rs::Sequence<Bspline>) -> bool;
}

// Corresponds to scan_planner_msgs__msg__Bspline
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Bspline {

    // This member is not documented.
    #[allow(missing_docs)]
    pub order: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub traj_id: i64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_time: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub knots: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos_pts: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_pts: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_dt: f64,

}



impl Default for Bspline {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !scan_planner_msgs__msg__Bspline__init(&mut msg as *mut _) {
        panic!("Call to scan_planner_msgs__msg__Bspline__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Bspline {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { scan_planner_msgs__msg__Bspline__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { scan_planner_msgs__msg__Bspline__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { scan_planner_msgs__msg__Bspline__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Bspline {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Bspline where Self: Sized {
  const TYPE_NAME: &'static str = "scan_planner_msgs/msg/Bspline";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__scan_planner_msgs__msg__Bspline() }
  }
}


#[link(name = "scan_planner_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__scan_planner_msgs__msg__DataDisp() -> *const std::ffi::c_void;
}

#[link(name = "scan_planner_msgs__rosidl_generator_c")]
extern "C" {
    fn scan_planner_msgs__msg__DataDisp__init(msg: *mut DataDisp) -> bool;
    fn scan_planner_msgs__msg__DataDisp__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DataDisp>, size: usize) -> bool;
    fn scan_planner_msgs__msg__DataDisp__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DataDisp>);
    fn scan_planner_msgs__msg__DataDisp__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DataDisp>, out_seq: *mut rosidl_runtime_rs::Sequence<DataDisp>) -> bool;
}

// Corresponds to scan_planner_msgs__msg__DataDisp
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DataDisp {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub a: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub b: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub c: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub d: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub e: f64,

}



impl Default for DataDisp {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !scan_planner_msgs__msg__DataDisp__init(&mut msg as *mut _) {
        panic!("Call to scan_planner_msgs__msg__DataDisp__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DataDisp {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { scan_planner_msgs__msg__DataDisp__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { scan_planner_msgs__msg__DataDisp__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { scan_planner_msgs__msg__DataDisp__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DataDisp {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DataDisp where Self: Sized {
  const TYPE_NAME: &'static str = "scan_planner_msgs/msg/DataDisp";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__scan_planner_msgs__msg__DataDisp() }
  }
}


#[link(name = "scan_planner_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__scan_planner_msgs__msg__TaskAuthorization() -> *const std::ffi::c_void;
}

#[link(name = "scan_planner_msgs__rosidl_generator_c")]
extern "C" {
    fn scan_planner_msgs__msg__TaskAuthorization__init(msg: *mut TaskAuthorization) -> bool;
    fn scan_planner_msgs__msg__TaskAuthorization__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TaskAuthorization>, size: usize) -> bool;
    fn scan_planner_msgs__msg__TaskAuthorization__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TaskAuthorization>);
    fn scan_planner_msgs__msg__TaskAuthorization__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TaskAuthorization>, out_seq: *mut rosidl_runtime_rs::Sequence<TaskAuthorization>) -> bool;
}

// Corresponds to scan_planner_msgs__msg__TaskAuthorization
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 任务授权：取消后必须由规划端**显式重新授权**才允许执行。
///
/// 为什么需要 task_id 而不是只有一个 bool：
///   取消时执行端会立即本地锁止（不等规划端），但可能有一条**在取消之前发布、
///   延迟到达**的授权消息。仅凭 bool 无法区分它和新任务的授权，会重新放行旧任务。
///   task_id 由规划端每接受一个新任务自增，执行端只接受 task_id 大于
///   "已撤销到的编号"的授权，从而不会因延迟消息解锁。

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskAuthorization {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// true=授权执行（新任务被接受）；false=撤销
    pub active: bool,

    /// 规划端单调自增的任务编号；撤销时携带当前编号
    pub task_id: u32,

}



impl Default for TaskAuthorization {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !scan_planner_msgs__msg__TaskAuthorization__init(&mut msg as *mut _) {
        panic!("Call to scan_planner_msgs__msg__TaskAuthorization__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TaskAuthorization {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { scan_planner_msgs__msg__TaskAuthorization__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { scan_planner_msgs__msg__TaskAuthorization__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { scan_planner_msgs__msg__TaskAuthorization__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TaskAuthorization {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TaskAuthorization where Self: Sized {
  const TYPE_NAME: &'static str = "scan_planner_msgs/msg/TaskAuthorization";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__scan_planner_msgs__msg__TaskAuthorization() }
  }
}


