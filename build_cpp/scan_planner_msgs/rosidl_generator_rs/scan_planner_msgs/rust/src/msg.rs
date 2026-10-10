#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to scan_planner_msgs__msg__Bspline

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub start_time: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub knots: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos_pts: Vec<geometry_msgs::msg::Point>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_pts: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_dt: f64,

}



impl Default for Bspline {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Bspline::default())
  }
}

impl rosidl_runtime_rs::Message for Bspline {
  type RmwMsg = super::msg::rmw::Bspline;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        order: msg.order,
        traj_id: msg.traj_id,
        start_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.start_time)).into_owned(),
        knots: msg.knots.into(),
        pos_pts: msg.pos_pts
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        yaw_pts: msg.yaw_pts.into(),
        yaw_dt: msg.yaw_dt,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      order: msg.order,
      traj_id: msg.traj_id,
        start_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start_time)).into_owned(),
        knots: msg.knots.as_slice().into(),
        pos_pts: msg.pos_pts
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        yaw_pts: msg.yaw_pts.as_slice().into(),
      yaw_dt: msg.yaw_dt,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      order: msg.order,
      traj_id: msg.traj_id,
      start_time: builtin_interfaces::msg::Time::from_rmw_message(msg.start_time),
      knots: msg.knots
          .into_iter()
          .collect(),
      pos_pts: msg.pos_pts
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
      yaw_pts: msg.yaw_pts
          .into_iter()
          .collect(),
      yaw_dt: msg.yaw_dt,
    }
  }
}


// Corresponds to scan_planner_msgs__msg__DataDisp

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DataDisp {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DataDisp::default())
  }
}

impl rosidl_runtime_rs::Message for DataDisp {
  type RmwMsg = super::msg::rmw::DataDisp;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        a: msg.a,
        b: msg.b,
        c: msg.c,
        d: msg.d,
        e: msg.e,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      a: msg.a,
      b: msg.b,
      c: msg.c,
      d: msg.d,
      e: msg.e,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      a: msg.a,
      b: msg.b,
      c: msg.c,
      d: msg.d,
      e: msg.e,
    }
  }
}


// Corresponds to scan_planner_msgs__msg__TaskAuthorization
/// 任务授权：取消后必须由规划端**显式重新授权**才允许执行。
///
/// 为什么需要 task_id 而不是只有一个 bool：
///   取消时执行端会立即本地锁止（不等规划端），但可能有一条**在取消之前发布、
///   延迟到达**的授权消息。仅凭 bool 无法区分它和新任务的授权，会重新放行旧任务。
///   task_id 由规划端每接受一个新任务自增，执行端只接受 task_id 大于
///   "已撤销到的编号"的授权，从而不会因延迟消息解锁。

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskAuthorization {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// true=授权执行（新任务被接受）；false=撤销
    pub active: bool,

    /// 规划端单调自增的任务编号；撤销时携带当前编号
    pub task_id: u32,

}



impl Default for TaskAuthorization {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TaskAuthorization::default())
  }
}

impl rosidl_runtime_rs::Message for TaskAuthorization {
  type RmwMsg = super::msg::rmw::TaskAuthorization;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        active: msg.active,
        task_id: msg.task_id,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      active: msg.active,
      task_id: msg.task_id,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      active: msg.active,
      task_id: msg.task_id,
    }
  }
}


