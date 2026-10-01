# adapt_VI_26_Sentry

ROS 2 全向哨兵机器人从 Nav2 向 SCAN-Planner 迁移的调研、实施与仿真仓库（私有）。

本仓库同时是文档工作区和**代码实施仓库**：SCAN-Planner 的 ROS 2 源码已纳入 `src/`，
在其上完成全向哨兵适配，并用用户提供的 PCD 搭建了可运行的 RViz 闭环运动仿真。

## 当前状态

| 阶段 | 内容 | 状态 |
|---|---|---|
| S0 | 记录基线、PCD、外形、外参、限速 | 完成，见 [S0 基线](docs/migration/s0_baseline.md) |
| S1 | 全向适配：odom 朝向、取消强制对齐、全向碰撞包络 | 完成，见 [实施报告](docs/migration/implementation_report.md) |
| S2 | PCD + RViz 闭环仿真（速度积分反馈 odom） | 完成 |
| S3 | 三模式与失效场景验证 | 完成（第二轮修正后），见 [测试结果](docs/testing/s3_results.md) |
| — | Codex 第一轮审查 | 已完成，发现 4 项缺陷，已修正 |
| — | Codex 复审 | **待进行** |
| I1–I4 | 接入 VI_26_Sentry 实车 | 未开始，另获授权后进行 |

**尚未接实车，也尚未通过复审。** 本仓库所有输出都在 `/sentry_sim` 命名空间内，不存在通往 UART / 底盘
的路径；仿真中的加速度、地面过滤阈值等参数是仿真取值，不是实车标定结果。

RViz 图形交互在本环境**无法验证**（创建不了 OpenGL 上下文），当前状态是
**"无界面闭环已有运行记录；RViz 交互待验收"**。

## 快速开始

```bash
# 1) 隔离构建（ROS 2 Humble + colcon，产物只落在本仓库）
scripts/build.sh

# 2) 启动仿真（RViz 2D Goal Pose 指定目标）
scripts/run_sentry_sim.sh navi_mode:=1

# 其它模式
scripts/run_sentry_sim.sh navi_mode:=2 \
  keypoints_file:=$(pwd)/install/scan_planner/share/scan_planner/config/sentry_waypoints.yaml
scripts/run_sentry_sim.sh navi_mode:=3 \
  reference_path_file:=$(pwd)/install/scan_planner/share/scan_planner/config/sentry_reference_path.yaml

# 变体
scripts/run_sentry_sim.sh yaw_mode:=spin spin_rate:=0.5   # 边转边走
scripts/run_sentry_sim.sh start_rviz:=false               # 无界面
```

默认地图是 `~/pcd_map/rmuc2026_field.pcd`，可用 `pcd_map_file:=<路径>` 覆盖；
文件不存在时 launch 会直接报错，不会静默换成演示地图。

无界面环境下跑场景验证与证据收集：

```bash
scripts/make_test_maps.py --out-dir docs/testing/maps  # 生成受控合成地图
scripts/smoke_test.sh 1              # 节点/话题/频率冒烟
scripts/scenario.sh mode1_lateral    # 场景记录 + 判据，失败返回非零
scripts/scenario.sh gap_edge         # 受控碰撞：0.44 m 缺口，中心线不碰但车体边缘会碰
scripts/check_clearance.py --csv log/scenarios/mode1_lateral.csv   # 独立净空检查
scripts/summarize_launch_log.sh      # 生成可审查的日志摘要
```

## 目录

```text
AGENTS.md              工作区规则（改代码前先读）
build.md               最初的架构侦察任务书
docs/                  调研、架构、接口、迁移与测试文档（入口 docs/README.md）
scripts/               构建 / 启动 / 场景验证 / 独立净空检查
src/planner/           纳入本仓库的 SCAN-Planner 包（见下方来源）
src/simulator/         地图发布、局部雷达渲染、合成地图
```

`src/` 的来源与改动边界见 [实施报告](docs/migration/implementation_report.md)；
`artifacts/`、`log/`、`build/`、`install/` 不入库。

## 已知边界

- **三维导航的含义**：沿用 SCAN 的三维占据/空间避障与参考高度实现，不做自由 z 优化、
  跨层全局搜索或轮地接触规划。
- **地面处理是演示方案，不是正式方案**：`rmuc2026_field.pcd` 的地面高度散布约
  -0.06..0.22 m，当前做法是把地图整体下移并按**绝对高度删点**（实测删掉 71.9% 的点）。
  这会连带删除矮于约 0.30 m 的**真实**结构——`low_obstacle_cut` 场景已复现该后果。
  原始地图通过 `/sentry_sim/global_cloud_raw` 保留对照显示。
  正式的局部地面高度/地面分割**尚未实现**，该阈值不得带到实车。
- **加速度上限**：RM 的 MPPI 加速度上限为 UNKNOWN，仿真用的是示例值。
- **安全余量**：用户未给出，当前 `safety_margin = 0`。
- 详细限制与未测项见 [测试结果](docs/testing/s3_results.md)。
