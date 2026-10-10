// 影子门控（C++）：与 Python 版语义一致——只发布 cmd_vel_shadow，绝不碰 /cmd_vel。
// 门控：健康/机体/候选新鲜 + 有限性 + 未定义分量剔除 + 限幅 + 地图心跳；
// 地图停更 = 任务失效：发 planning/reset 并锁止，直到编号更大的新任务且地图恢复。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <utility>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/task_authorization.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

namespace sentry_scan_adapter_cpp {

class ShadowGuard : public rclcpp::Node {
 public:
  ShadowGuard() : Node("shadow_guard") {
    health_topic_ = declare_parameter<std::string>("health_topic", "health_ok");
    candidate_topic_ = declare_parameter<std::string>("candidate_topic", "cmd_vel_candidate");
    shadow_topic_ = declare_parameter<std::string>("shadow_topic", "cmd_vel_shadow");
    body_pose_topic_ = declare_parameter<std::string>("body_pose_topic", "body_pose");
    cloud_update_topic_ =
        declare_parameter<std::string>("cloud_update_topic", "grid_map/cloud_update");
    task_active_topic_ =
        declare_parameter<std::string>("task_active_topic", "planning/task_active");
    max_map_age_ = declare_parameter<double>("max_map_age", 0.5);
    revoke_repeat_period_ = declare_parameter<double>("revoke_repeat_period", 1.0);
    max_health_age_ = declare_parameter<double>("max_health_age", 0.5);
    max_body_age_ = declare_parameter<double>("max_body_age", 0.5);
    max_vx_ = declare_parameter<double>("max_vx", 1.0);
    max_vy_ = declare_parameter<double>("max_vy", 1.0);
    max_wz_ = declare_parameter<double>("max_wz", 1.5);
    zero_yaw_candidate_ = declare_parameter<bool>("zero_yaw_candidate", true);
    output_rate_ = declare_parameter<double>("output_rate", 20.0);
    record_ = declare_parameter<bool>("record", true);
    graph_check_period_ = declare_parameter<double>("graph_check_period", 1.0);
    log_dir_ = declare_parameter<std::string>("log_dir", "log/shadow");

    auto latch = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    // 必须用成员变量持有：rclcpp 的订阅/定时器由返回的 shared_ptr 拥有，
    // 构造结束即丢弃会让订阅失效（曾导致 guard 收不到 health/心跳，恒判 map_update_stale）。
    health_sub_ = create_subscription<std_msgs::msg::Bool>(
        health_topic_, latch,
        [this](std_msgs::msg::Bool::SharedPtr msg) { health_ = msg->data; health_recv_ = now_s(); });
    candidate_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        candidate_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          candidate_ = *msg;
          candidate_recv_ = now_s();
        });
    body_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        body_pose_topic_, 10,
        [this](nav_msgs::msg::Odometry::SharedPtr) { body_recv_ = now_s(); });
    map_sub_ = create_subscription<std_msgs::msg::Header>(
        cloud_update_topic_, 10, [this](std_msgs::msg::Header::SharedPtr msg) {
          map_recv_ = now_s();
          // 源时间戳同样要新鲜：持续收到旧 stamp 的心跳不能算"地图在更新"
          map_stamp_ = static_cast<double>(msg->stamp.sec) +
                       static_cast<double>(msg->stamp.nanosec) * 1e-9;
          map_ever_fresh_ = true;
        });
    task_sub_ = create_subscription<scan_planner_msgs::msg::TaskAuthorization>(
        task_active_topic_, latch,
        [this](scan_planner_msgs::msg::TaskAuthorization::SharedPtr msg) { on_task(msg); });

    shadow_pub_ = create_publisher<geometry_msgs::msg::Twist>(shadow_topic_, 10);
    reset_pub_ = create_publisher<std_msgs::msg::Bool>("planning/reset", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / output_rate_),
                               [this]() { tick(); });
    graph_timer_ = create_wall_timer(std::chrono::duration<double>(graph_check_period_),
                                     [this]() { check_graph(); });
    if (record_) {
      open_csv();
    }
    RCLCPP_WARN(get_logger(),
                "Shadow guard (C++) ready: candidate='%s' -> shadow='%s'. This node creates NO "
                "publisher on /cmd_vel. Map heartbeat='%s' (max age %.2fs; stale map always "
                "stops and is latched until a new task).",
                candidate_topic_.c_str(), shadow_topic_.c_str(), cloud_update_topic_.c_str(),
                max_map_age_);
  }

  ~ShadowGuard() override {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

 private:
  double now_s() { return get_clock()->now().seconds(); }

  void open_csv() {
    try {
      std::string dir = log_dir_;
      if (!dir.empty() && dir.back() == '/') {
        dir.pop_back();
      }
      std::string command = "mkdir -p '" + dir + "'";
      if (std::system(command.c_str()) != 0) {
        return;
      }
      const auto now = std::chrono::system_clock::now();
      const std::time_t stamp = std::chrono::system_clock::to_time_t(now);
      char buffer[32];
      std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&stamp));
      csv_path_ = dir + "/shadow_" + buffer + ".csv";
      csv_.open(csv_path_);
      csv_ << "wall_time,ros_time,health,gate,cand_vx,cand_vy,cand_wz,out_vx,out_vy,out_wz,"
              "map_age,map_latched,external_cmdvel_pubs,graph_violation\n";
    } catch (const std::exception &) {
    }
  }

  std::string map_stale_reason(double now) {
    if (map_recv_ <= 0.0 || (now - map_recv_) > max_map_age_) {
      return "map_update_stale";
    }
    if (map_stamp_ > 0.0 && (now - map_stamp_) > max_map_age_) {
      return "map_update_stamp_stale";
    }
    return "";
  }

  // 影子运行时的图隔离检查：允许外部 Nav2/UART 并存，只判定**影子命名空间**的
  // 违规控制发布者；图查询失败不能记成"安全"。
  void check_graph() {
    const std::string own_ns = std::string(get_namespace());
    int external = 0;
    std::string violation;
    for (const auto &topic : {std::string("/cmd_vel"), std::string("/cmd_vel_remap")}) {
      try {
        for (const auto &info : get_publishers_info_by_topic(topic)) {
          std::string namespace_ = info.node_namespace();
          while (!namespace_.empty() && namespace_.back() == '/') {
            namespace_.pop_back();
          }
          if (namespace_ == own_ns) {
            violation = topic + " published by shadow node " + info.node_name();
          } else {
            external++;
          }
        }
      } catch (const std::exception &exc) {
        violation = "graph query failed for " + topic + ": " + exc.what();
      }
    }
    external_publishers_ = external;
    graph_violation_ = violation;
    if (!violation.empty()) {
      RCLCPP_ERROR(get_logger(), "shadow graph violation: %s", violation.c_str());
    }
  }

  void publish_reset(const std::string &reason) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    reset_pub_->publish(msg);
    last_revoke_ = now_s();
    RCLCPP_ERROR(get_logger(),
                 "Map update NOT healthy (%s): publishing planning/reset and LATCHING the shadow "
                 "output; only a new task (task_id > %u) with a recovered map will resume",
                 reason.c_str(), latched_task_id_);
  }

  void on_task(const scan_planner_msgs::msg::TaskAuthorization::SharedPtr msg) {
    if (msg->task_id > last_task_id_) {
      last_task_id_ = msg->task_id;
    }
    if (!msg->active) {
      task_active_ = false;
      return;
    }
    task_active_ = true;
    if (!map_latched_) {
      return;
    }
    if (msg->task_id <= latched_task_id_) {
      RCLCPP_WARN(get_logger(), "Authorization task_id=%u is not newer than the latched id=%u; "
                                "map latch kept", msg->task_id, latched_task_id_);
      return;
    }
    if (!map_stale_reason(now_s()).empty()) {
      RCLCPP_WARN(get_logger(), "New task %u authorized but the map is still stale; latch kept",
                  msg->task_id);
      return;
    }
    map_latched_ = false;
    RCLCPP_WARN(get_logger(), "New task %u authorized after the map recovered; map latch cleared "
                              "(old task was revoked)", msg->task_id);
  }

  std::pair<geometry_msgs::msg::Twist, std::string> decide() {
    geometry_msgs::msg::Twist zero;
    const double now = now_s();
    if (!graph_violation_.empty()) {
      return {zero, "graph_violation"};
    }
    const std::string stale = map_stale_reason(now);
    if (!stale.empty() && map_ever_fresh_ && task_active_ && !map_latched_) {
      map_latched_ = true;
      latched_task_id_ = last_task_id_;
      publish_reset(stale);
    }
    if (map_latched_) {
      if ((now - last_revoke_) >= revoke_repeat_period_) {
        publish_reset(stale.empty() ? "map_latched" : stale);
      }
      return {zero, "map_latched"};
    }
    if (!stale.empty()) {
      return {zero, stale};
    }
    if (!health_) {
      return {zero, "inputs_unhealthy"};
    }
    if (health_recv_ <= 0.0 || (now - health_recv_) > max_health_age_) {
      return {zero, "health_stale"};
    }
    if (body_recv_ <= 0.0 || (now - body_recv_) > max_body_age_) {
      return {zero, "body_pose_stale"};
    }
    if (candidate_recv_ <= 0.0 || (now - candidate_recv_) > max_health_age_) {
      return {zero, "candidate_stale"};
    }
    const double values[] = {candidate_.linear.x, candidate_.linear.y, candidate_.linear.z,
                             candidate_.angular.x, candidate_.angular.y, candidate_.angular.z};
    for (const double value : values) {
      if (!std::isfinite(value)) {
        return {zero, "candidate_non_finite"};
      }
    }
    if (std::fabs(candidate_.linear.z) > 1e-9 || std::fabs(candidate_.angular.x) > 1e-9 ||
        std::fabs(candidate_.angular.y) > 1e-9) {
      return {zero, "candidate_unsupported_axes"};
    }
    geometry_msgs::msg::Twist out;
    out.linear.x = std::clamp(candidate_.linear.x, -max_vx_, max_vx_);
    out.linear.y = std::clamp(candidate_.linear.y, -max_vy_, max_vy_);
    out.angular.z = zero_yaw_candidate_
                        ? 0.0
                        : std::clamp(candidate_.angular.z, -max_wz_, max_wz_);
    const std::string reason =
        (zero_yaw_candidate_ && std::fabs(candidate_.angular.z) > 1e-9) ? "pass_yaw_zeroed"
                                                                        : "pass";
    return {out, reason};
  }

  void tick() {
    auto [command, reason] = decide();
    shadow_pub_->publish(command);
    samples_++;
    if (csv_.is_open() && (samples_ % 5 == 0 || reason != "pass")) {
      const double now = now_s();
      csv_ << now << ',' << now << ',' << (health_ ? 1 : 0) << ',' << reason << ','
           << candidate_.linear.x << ',' << candidate_.linear.y << ',' << candidate_.angular.z
           << ',' << command.linear.x << ',' << command.linear.y << ',' << command.angular.z
           << ',' << (map_recv_ > 0.0 ? now - map_recv_ : -1.0) << ',' << (map_latched_ ? 1 : 0)
           << ',' << external_publishers_ << ',' << graph_violation_ << '\n';
    }
  }

  std::string health_topic_, candidate_topic_, shadow_topic_, body_pose_topic_;
  std::string cloud_update_topic_, task_active_topic_, log_dir_, csv_path_;
  double max_map_age_{}, revoke_repeat_period_{}, max_health_age_{}, max_body_age_{};
  double max_vx_{}, max_vy_{}, max_wz_{}, output_rate_{};
  bool zero_yaw_candidate_{}, record_{};
  bool health_{false}, map_ever_fresh_{false}, map_latched_{false}, task_active_{false};
  double health_recv_{0.0}, body_recv_{0.0}, map_recv_{0.0}, map_stamp_{0.0};
  double candidate_recv_{0.0}, last_revoke_{0.0}, graph_check_period_{1.0};
  std::string graph_violation_;
  unsigned int last_task_id_{0}, latched_task_id_{0};
  long samples_{0};
  int external_publishers_{0};
  geometry_msgs::msg::Twist candidate_;
  std::ofstream csv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr shadow_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;
  rclcpp::TimerBase::SharedPtr timer_, graph_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr candidate_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr body_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr map_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::TaskAuthorization>::SharedPtr task_sub_;
};

}  // namespace sentry_scan_adapter_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_scan_adapter_cpp::ShadowGuard>());
  rclcpp::shutdown();
  return 0;
}
