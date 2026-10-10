// 命令闸门（C++）：SCAN 候选速度与底盘话题之间唯一的安全开关，默认不输出。
//
// 与 Python 版 `sentry_scan_adapter/cmd_gate.py` 语义一致：
//   默认禁止输出（从未使能时不发布任何消息）→ 显式使能 → 任务授权 + 输入健康 + 地图心跳新鲜
//   + 候选新鲜 + 限幅非零，五者齐全才透传；`planning/reset` 会清零并**清除操作者使能**。
// 输出话题默认 `cmd_vel_gated`（不是 `/cmd_vel`），误启动也不会碰到真实底盘入口。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <tuple>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/task_authorization.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace sentry_scan_adapter_cpp {

class CmdGate : public rclcpp::Node {
 public:
  CmdGate() : Node("cmd_gate") {
    candidate_topic_ = declare_parameter<std::string>("candidate_topic", "cmd_vel_candidate");
    output_topic_ = declare_parameter<std::string>("output_topic", "cmd_vel_gated");
    enable_topic_ = declare_parameter<std::string>("enable_topic", "cmd_vel_enable");
    health_topic_ = declare_parameter<std::string>("health_topic", "health_ok");
    task_active_topic_ =
        declare_parameter<std::string>("task_active_topic", "planning/task_active");
    map_update_topic_ =
        declare_parameter<std::string>("map_update_topic", "grid_map/cloud_update");
    state_topic_ = declare_parameter<std::string>("state_topic", "cmd_gate/state");
    candidate_timeout_ = declare_parameter<double>("candidate_timeout", 0.5);
    max_map_age_ = declare_parameter<double>("max_map_age", 0.5);
    max_health_age_ = declare_parameter<double>("max_health_age", 0.5);
    // 默认 0：不显式设置上限就只会输出零（默认拒绝的第二把锁）
    max_vx_ = declare_parameter<double>("max_vx", 0.0);
    max_vy_ = declare_parameter<double>("max_vy", 0.0);
    max_wz_ = declare_parameter<double>("max_wz", 0.0);
    allow_wz_ = declare_parameter<bool>("allow_wz", false);
    publish_period_ = declare_parameter<double>("publish_period", 0.02);

    auto latch = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    candidate_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        candidate_topic_, 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { on_candidate(msg); });
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        enable_topic_, latch, [this](std_msgs::msg::Bool::SharedPtr msg) { on_enable(msg); });
    health_sub_ = create_subscription<std_msgs::msg::Bool>(
        health_topic_, latch, [this](std_msgs::msg::Bool::SharedPtr msg) { on_health(msg); });
    task_sub_ = create_subscription<scan_planner_msgs::msg::TaskAuthorization>(
        task_active_topic_, latch,
        [this](scan_planner_msgs::msg::TaskAuthorization::SharedPtr msg) { on_task(msg); });
    map_sub_ = create_subscription<std_msgs::msg::Header>(
        map_update_topic_, 10, [this](std_msgs::msg::Header::SharedPtr) {
          map_recv_ = now_s();
        });
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
        "planning/reset", 10,
        [this](std_msgs::msg::Bool::SharedPtr msg) { on_reset(msg); });

    pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, latch);
    timer_ = create_wall_timer(std::chrono::duration<double>(publish_period_),
                               [this]() { tick(); });

    RCLCPP_WARN(get_logger(),
                "Command gate (C++) ready: '%s' -> '%s' (DEFAULT DISABLED). Enable via '%s'; "
                "limits max_vx=%.2f max_vy=%.2f max_wz=%.2f allow_wz=%s. planning/reset clears "
                "the enable.",
                candidate_topic_.c_str(), output_topic_.c_str(), enable_topic_.c_str(), max_vx_,
                max_vy_, max_wz_, allow_wz_ ? "true" : "false");
    if (max_vx_ == 0.0 && max_vy_ == 0.0) {
      RCLCPP_ERROR(get_logger(),
                   "max_vx/max_vy are 0: the gate will only ever publish zero. Set explicit "
                   "limits at launch before any takeover test.");
    }
  }

 public:  // 状态与 decide() 对确定性单测（gtest）可见（与 Python 版一致）
  double now_s() { return get_clock()->now().seconds(); }

  void on_candidate(const geometry_msgs::msg::Twist::SharedPtr msg) {
    candidate_ = *msg;
    candidate_recv_ = now_s();
  }

  void on_enable(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool desired = msg->data;
    if (desired && !armed_) {
      RCLCPP_ERROR(get_logger(),
                   "Command gate ENABLED by operator: output '%s' may now be published.",
                   output_topic_.c_str());
    } else if (!desired && armed_) {
      RCLCPP_WARN(get_logger(), "Command gate disabled by operator; output returns to zero.");
    }
    armed_ = desired;
    ever_armed_ = ever_armed_ || desired;
  }

  void on_health(const std_msgs::msg::Bool::SharedPtr msg) {
    health_ = msg->data;
    health_recv_ = now_s();
  }

  void on_task(const scan_planner_msgs::msg::TaskAuthorization::SharedPtr msg) {
    if (msg->task_id > last_task_id_) {
      last_task_id_ = msg->task_id;
    }
    if (msg->active && msg->task_id > cancelled_task_id_) {
      authorized_ = true;
    } else {
      authorized_ = false;
    }
  }

  void on_reset(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) {
      return;
    }
    authorized_ = false;
    cancelled_task_id_ = last_task_id_;
    if (armed_) {
      armed_ = false;
      RCLCPP_ERROR(get_logger(),
                   "planning/reset received: gate latched to zero and the operator enable was "
                   "CLEARED. Re-enable explicitly ('%s').",
                   enable_topic_.c_str());
    }
  }

  // 返回 {是否发布, 指令, 状态}
  std::tuple<bool, geometry_msgs::msg::Twist, std::string> decide() {
    geometry_msgs::msg::Twist zero;
    const double now = now_s();
    if (!ever_armed_) {
      return {false, zero, "disabled(never-enabled)"};
    }
    if (!armed_) {
      return {true, zero, "zero(disabled)"};
    }
    if (!authorized_) {
      return {true, zero, "zero(no-task-authorization)"};
    }
    if (health_ != 1) {
      return {true, zero, "zero(inputs-unhealthy)"};
    }
    if (health_recv_ <= 0.0 || (now - health_recv_) > max_health_age_) {
      return {true, zero, "zero(health-stale)"};
    }
    if (map_recv_ <= 0.0 || (now - map_recv_) > max_map_age_) {
      return {true, zero, "zero(map-stale)"};
    }
    if (candidate_recv_ <= 0.0 || (now - candidate_recv_) > candidate_timeout_) {
      return {true, zero, "zero(candidate-stale)"};
    }
    double vx = candidate_.linear.x;
    double vy = candidate_.linear.y;
    double wz = candidate_.angular.z;
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
      return {true, zero, "zero(non-finite-candidate)"};
    }
    vx = std::clamp(vx, -max_vx_, max_vx_);
    vy = std::clamp(vy, -max_vy_, max_vy_);
    wz = allow_wz_ ? std::clamp(wz, -max_wz_, max_wz_) : 0.0;
    if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
      return {true, zero, "zero(limits-or-candidate-zero)"};
    }
    geometry_msgs::msg::Twist out;
    out.linear.x = vx;
    out.linear.y = vy;
    out.angular.z = wz;
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), "active(vx=%.3f,vy=%.3f,wz=%.3f)", vx, vy, wz);
    return {true, out, buffer};
  }

  void tick() {
    auto [publish, command, state] = decide();
    if (publish) {
      pub_->publish(command);
    }
    if (state != last_state_) {
      last_state_ = state;
      std_msgs::msg::String msg;
      msg.data = state;
      state_pub_->publish(msg);
      RCLCPP_WARN(get_logger(), "cmd gate state: %s", state.c_str());
    }
  }

  std::string candidate_topic_, output_topic_, enable_topic_, health_topic_;
  std::string task_active_topic_, map_update_topic_, state_topic_;
  double candidate_timeout_{}, max_map_age_{}, max_health_age_{};
  double max_vx_{}, max_vy_{}, max_wz_{};
  bool allow_wz_{};
  double publish_period_{};

  bool armed_{false}, ever_armed_{false}, authorized_{false};
  unsigned int last_task_id_{0}, cancelled_task_id_{0};
  int health_{0};
  double health_recv_{0.0}, map_recv_{0.0}, candidate_recv_{0.0};
  geometry_msgs::msg::Twist candidate_;
  std::string last_state_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr candidate_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_, health_sub_, reset_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::TaskAuthorization>::SharedPtr task_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentry_scan_adapter_cpp

#ifndef CMD_GATE_UNIT_TEST
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_scan_adapter_cpp::CmdGate>());
  rclcpp::shutdown();
  return 0;
}
#endif
