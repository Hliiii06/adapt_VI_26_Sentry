// 命令闸门互锁矩阵的确定性单测（gtest）。
// 覆盖接管要求：默认不输出 / 显式使能 / 任务授权 / 健康 / 地图心跳 / 候选新鲜 / 限幅 / 取消清除使能。
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>

#define CMD_GATE_UNIT_TEST
#include "cmd_gate_node.cpp"

using sentry_scan_adapter_cpp::CmdGate;

namespace sentry_scan_adapter_cpp {

class CmdGateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<CmdGate>();
    node_->max_vx_ = 0.3;
    node_->max_vy_ = 0.3;
    node_->max_wz_ = 0.5;
    reset_state();
  }

  void TearDown() override {
    node_.reset();
    rclcpp::shutdown();
  }

  void reset_state() {
    const double now = node_->now_s();
    node_->armed_ = true;
    node_->ever_armed_ = true;
    node_->authorized_ = true;
    node_->last_task_id_ = 3;
    node_->cancelled_task_id_ = 0;
    node_->health_ = 1;
    node_->health_recv_ = now;
    node_->map_recv_ = now;
    node_->candidate_ = geometry_msgs::msg::Twist();
    node_->candidate_.linear.x = 1.0;
    node_->candidate_.linear.y = 0.5;
    node_->candidate_.angular.z = 0.9;
    node_->candidate_recv_ = now;
  }

  std::shared_ptr<CmdGate> node_;
};

TEST_F(CmdGateTest, NeverEnabledPublishesNothing) {
  node_->ever_armed_ = false;
  auto [publish, command, state] = node_->decide();
  EXPECT_FALSE(publish);
  EXPECT_EQ(state, "disabled(never-enabled)");
}

TEST_F(CmdGateTest, RequiresTaskAuthorization) {
  node_->authorized_ = false;
  auto [publish, command, state] = node_->decide();
  EXPECT_TRUE(publish);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_EQ(state, "zero(no-task-authorization)");
}

TEST_F(CmdGateTest, ClampsAndZeroesRotation) {
  auto [publish, command, state] = node_->decide();
  EXPECT_TRUE(publish);
  EXPECT_NEAR(command.linear.x, 0.3, 1e-9);
  EXPECT_NEAR(command.linear.y, 0.3, 1e-9);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);  // allow_wz 默认 false
  EXPECT_EQ(state.rfind("active", 0), 0u);
}

TEST_F(CmdGateTest, ZeroLimitsBlockEverything) {
  node_->max_vx_ = 0.0;
  node_->max_vy_ = 0.0;
  auto [publish, command, state] = node_->decide();
  EXPECT_TRUE(publish);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_EQ(state, "zero(limits-or-candidate-zero)");
}

TEST_F(CmdGateTest, CandidateStaleStops) {
  node_->candidate_recv_ = node_->now_s() - 5.0;
  auto [publish, command, state] = node_->decide();
  EXPECT_TRUE(publish);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_EQ(state, "zero(candidate-stale)");
}

TEST_F(CmdGateTest, UnhealthyInputsStop) {
  node_->health_ = 0;
  auto [publish, command, state] = node_->decide();
  EXPECT_TRUE(publish);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_EQ(state, "zero(inputs-unhealthy)");
}

TEST_F(CmdGateTest, StaleMapStops) {
  node_->map_recv_ = node_->now_s() - 5.0;
  auto [publish, command, state] = node_->decide();
  EXPECT_TRUE(publish);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_EQ(state, "zero(map-stale)");
}

TEST_F(CmdGateTest, NonFiniteCandidateStops) {
  node_->candidate_.linear.x = std::nan("");
  auto [publish, command, state] = node_->decide();
  EXPECT_TRUE(publish);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_EQ(state, "zero(non-finite-candidate)");
}

TEST_F(CmdGateTest, RotationPassthroughWhenAllowed) {
  node_->allow_wz_ = true;
  auto [publish, command, state] = node_->decide();
  EXPECT_NEAR(command.angular.z, 0.5, 1e-9);
}

TEST_F(CmdGateTest, NewTaskAfterCancelNeedsReEnable) {
  std_msgs::msg::Bool reset;
  reset.data = true;
  node_->on_reset(std::make_shared<std_msgs::msg::Bool>(reset));
  auto task = std::make_shared<scan_planner_msgs::msg::TaskAuthorization>();
  task->active = true;
  task->task_id = 4;
  node_->on_task(task);
  EXPECT_TRUE(node_->authorized_);
  EXPECT_FALSE(node_->armed_);
  auto [publish, command, state] = node_->decide();
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  std_msgs::msg::Bool enable;
  enable.data = true;
  node_->on_enable(std::make_shared<std_msgs::msg::Bool>(enable));
  auto [publish2, command2, state2] = node_->decide();
  EXPECT_NEAR(command2.linear.x, 0.3, 1e-9);
}

TEST_F(CmdGateTest, ResetClearsEnable) {
  std_msgs::msg::Bool reset;
  reset.data = true;
  node_->on_reset(std::make_shared<std_msgs::msg::Bool>(reset));
  EXPECT_FALSE(node_->armed_);
  EXPECT_FALSE(node_->authorized_);
  auto [publish, command, state] = node_->decide();
  EXPECT_TRUE(publish);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_EQ(state, "zero(disabled)");
}

}  // namespace sentry_scan_adapter_cpp

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
