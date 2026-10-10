// 影子专用输入暂停闸门（C++）：插在实车话题与适配器之间，暂停时只停止转发。
// 只订阅实车话题，只发布 /sentry_scan/*（test/odom、test/velocity、test/cloud、标记、状态）。
#include <chrono>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

namespace sentry_scan_adapter_cpp {

class InputPauseGate : public rclcpp::Node {
 public:
  InputPauseGate() : Node("input_pause_gate") {
    odom_in_ = declare_parameter<std::string>("odom_in", "/Odometry_transformed");
    velocity_in_ = declare_parameter<std::string>("velocity_in", "/LIVO2/imu_propagate");
    cloud_in_ = declare_parameter<std::string>("cloud_in", "/cloud_registered");
    odom_out_ = declare_parameter<std::string>("odom_out", "test/odom");
    velocity_out_ = declare_parameter<std::string>("velocity_out", "test/velocity");
    cloud_out_ = declare_parameter<std::string>("cloud_out", "test/cloud");
    pause_topic_ = declare_parameter<std::string>("pause_topic", "test/pause_inputs");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "test/fault_marker");
    pause_after_ = declare_parameter<double>("pause_after", 0.0);
    resume_after_ = declare_parameter<double>("resume_after", 0.0);

    const auto qos = rclcpp::SensorDataQoS();
    create_subscription<nav_msgs::msg::Odometry>(odom_in_, qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          if (paused_) { dropped_++; return; } forwarded_++; odom_pub_->publish(*msg);
        });
    create_subscription<nav_msgs::msg::Odometry>(velocity_in_, qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          if (paused_) { dropped_++; return; } forwarded_++; velocity_pub_->publish(*msg);
        });
    create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in_, qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          if (paused_) { dropped_++; return; } forwarded_++; cloud_pub_->publish(*msg);
        });
    create_subscription<std_msgs::msg::Bool>(pause_topic_, 10,
        [this](std_msgs::msg::Bool::SharedPtr msg) { set_paused(msg->data, "operator command"); });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_out_, qos);
    velocity_pub_ = create_publisher<nav_msgs::msg::Odometry>(velocity_out_, qos);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_, qos);
    marker_pub_ = create_publisher<std_msgs::msg::Header>(marker_topic_, 10);
    auto latch = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    paused_pub_ = create_publisher<std_msgs::msg::Bool>("test/input_paused", latch);
    if (pause_after_ > 0.0 || resume_after_ > 0.0) {
      timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { automate(); });
      start_ = now_s();
    }
    publish_state("ready");
    RCLCPP_WARN(get_logger(),
                "Input pause gate (C++): %s -> %s, %s -> %s, %s -> %s; pause via '%s' "
                "(true=pause). Pausing only stops forwarding to /sentry_scan.",
                odom_in_.c_str(), odom_out_.c_str(), velocity_in_.c_str(), velocity_out_.c_str(),
                cloud_in_.c_str(), cloud_out_.c_str(), pause_topic_.c_str());
  }

 private:
  double now_s() { return get_clock()->now().seconds(); }

  void publish_state(const std::string &marker) {
    std_msgs::msg::Bool state;
    state.data = paused_;
    paused_pub_->publish(state);
    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    header.frame_id = marker;
    marker_pub_->publish(header);
  }

  void set_paused(bool paused, const std::string &reason) {
    if (paused == paused_) {
      return;
    }
    paused_ = paused;
    RCLCPP_WARN(get_logger(), "%s inputs (%s); forwarded=%ld dropped=%ld",
                paused ? "PAUSED" : "RESUMED", reason.c_str(), forwarded_, dropped_);
    publish_state(paused ? "input_paused" : "input_resumed");
  }

  void automate() {
    const double elapsed = now_s() - start_;
    if (pause_after_ > 0.0 && !paused_ && elapsed >= pause_after_) {
      set_paused(true, "pause_after");
    }
    if (resume_after_ > 0.0 && paused_ && elapsed >= resume_after_) {
      set_paused(false, "resume_after");
    }
  }

  std::string odom_in_, velocity_in_, cloud_in_, odom_out_, velocity_out_, cloud_out_;
  std::string pause_topic_, marker_topic_;
  double pause_after_{}, resume_after_{}, start_{0.0};
  bool paused_{false};
  long forwarded_{0}, dropped_{0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, velocity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr paused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentry_scan_adapter_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_scan_adapter_cpp::InputPauseGate>());
  rclcpp::shutdown();
  return 0;
}
