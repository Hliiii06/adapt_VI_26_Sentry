# adapt_VI_26_Sentry

ROS 2 全向哨兵机器人从 Nav2 向 SCAN-Planner 迁移的调研、实施与仿真仓库（私有）。

本仓库同时是文档工作区和**代码实施仓库**：SCAN-Planner 的 ROS 2 源码已纳入 `src/`，
在其上完成全向哨兵适配，并用用户提供的 PCD 搭建了可运行的 RViz 闭环运动仿真。

## 当前状态

**2026-10-07：暂停扩建仿真，转向实车接入准备。** 下一轮先按
[整理指引](docs/migration/cleanup_handoff.md)收敛代码与文档，再按
[I1/I2 影子接入交接](docs/migration/real_robot_handoff.md)接入 RM 数据；尚不输出底盘命令。
下文为已完成的仿真阶段记录。

2026-10-02：已修复 A* 下坡取整和模拟雷达虚增洞顶厚度，并实现用户批准的指定通道支撑层选择。
真实 PCD 大坡在半径 0.26 / 高度 0.25 m 下到达；两个洞口及后坡在高度 0.10 m 下到达，
同图高度 0.25 m 拒绝。**不是整场地形规划或实车过洞认证**。
启动入口与本轮证据见 [真实 PCD 路线修复](docs/testing/pcd_route_fix.md)。下表为此前阶段记录。

| 阶段 | 内容 | 状态 |
|---|---|---|
| S0 | 记录基线、PCD、外形、外参、限速 | 完成，见 [S0 基线](docs/migration/s0_baseline.md) |
| S1 | 全向适配：odom 朝向、取消强制对齐、全向碰撞包络 | 完成，见 [实施报告](docs/migration/implementation_report.md) |
| S2 | PCD + RViz 闭环仿真（速度积分反馈 odom） | 完成 |
| S3 | 三模式与失效场景验证 | 完成（第二轮修正后），见 [测试结果](docs/testing/s3_results.md) |
| T | 地形分离与高度跟随（RViz 可见坡度） | 完成，见 [地形与高度跟随](docs/testing/terrain_following.md) |
| — | Codex 第一轮审查 | 已完成，发现 4 项缺陷，已修正 |
| — | Codex 复审 | 已完成，5 项基础缺陷 + 遗留项已修正；见 [地形核对](docs/testing/field_terrain_check.md) |
| — | Codex 第二轮复审 | 已完成，4 项实现问题已修正；见 [第二轮修正](docs/testing/review_round2_fixes.md) |
| T2 | 受控洞口场景（高洞过 / 低洞拒） | 完成，见 [真实场地地形核对](docs/testing/field_terrain_check.md) |
| T3 | RViz 实体显示（地形网格 + 实体机体） | 已实现，**观感待用户验收** |
| I1–I4 | 接入 VI_26_Sentry 实车 | 未开始，另获授权后进行 |

**尚未接实车。** 本仓库所有输出都在 `/sentry_sim` 命名空间内，不存在通往 UART / 底盘
的路径；仿真中的加速度、地面过滤阈值等参数是仿真取值，不是实车标定结果。

RViz 图形交互在本环境**无法验证**（创建不了 OpenGL 上下文），当前状态是
**"无界面闭环已有运行记录；RViz 交互待验收"**。

## 快速开始

```bash
# 1) 隔离构建（ROS 2 Humble + colcon，产物只落在本仓库）
scripts/build.sh

# 2) 启动仿真（RViz 2D Goal Pose 指定目标）
scripts/run_sentry_sim.sh navi_mode:=1

# 指定真实坡道：目标 (9, 4.2)，保留车高 0.25 m
bash scripts/run_field_route.sh large_ramp
# 指定真实洞口及后坡：目标 (4.7, 6.1)，仅降高可通行性实验
bash scripts/run_field_route.sh small_tunnel robot_height:=0.10

# 其它模式
scripts/run_sentry_sim.sh navi_mode:=2 \
  keypoints_file:=$(pwd)/install/scan_planner/share/scan_planner/config/sentry_waypoints.yaml
scripts/run_sentry_sim.sh navi_mode:=3 \
  reference_path_file:=$(pwd)/install/scan_planner/share/scan_planner/config/sentry_reference_path.yaml

# 变体
scripts/run_sentry_sim.sh yaw_mode:=spin spin_rate:=0.5   # 边转边走
scripts/run_sentry_sim.sh start_rviz:=false               # 无界面
```

**带地形（RViz 里能看见坡度）**：完整命令见
[地形与高度跟随](docs/testing/terrain_following.md) 第七节。要点是四个参数一起给：
`ground_file`（地形表面显示）、`ground_grid_file`（高度查询）、
`pcd_map_file`（只含障碍）、`map_offset_z:=0.0`。

默认地图是 `~/pcd_map/rmuc2026_field.pcd`，可用 `pcd_map_file:=<路径>` 覆盖；
文件不存在时 launch 会直接报错，不会静默换成演示地图。

无界面环境下跑场景验证与证据收集：

```bash
scripts/make_test_maps.py --out-dir docs/testing/maps          # 受控碰撞地图
scripts/make_terrain_maps.py --out-dir docs/testing/maps/terrain  # 合成坡道
scripts/prepare_terrain_map.py --input ~/pcd_map/rmuc2026_field.pcd \
  --out-prefix docs/testing/maps/field/rmuc2026                # 真实场地地形分离
scripts/smoke_test.sh 1              # 节点/话题/频率冒烟
scripts/scenario.sh mode1_lateral    # 场景记录 + 判据，失败返回非零
scripts/scenario.sh gap_edge         # 受控碰撞：0.44 m 缺口，中心线不碰但车体边缘会碰
scripts/check_clearance.py --csv log/scenarios/mode1_lateral.csv   # 独立净空检查
scripts/scenario.sh terrain_ramp20   # 合成 20° 坡的高度跟随
scripts/scenario.sh terrain_field    # 真实场地缓坡的高度跟随
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
- **地面处理仍有边界**：旧低分位数方法会选中坡面下的底板；最新入口仅在三条指定通道
  连续选择支撑层，其他区域沿用背景障碍分类，不是全场多层地形图。
  表面容差可能吞掉很矮结构，不可直接用于实车。旧绝对高度删点仍仅供演示。
- **真实坡面已确认**：本轮大坡沿指定路线升高 0.20 m。此前“可行驶起伏只有 0.14 m”
  的概括撤回；其他区域能否通行不能由这三条路线推断。
- **高度跟随没有动力学**：z 由地形网格推导，没有轮地接触/牵引/打滑模型，
  **不能当作实车爬坡验证**；优化后的控制点按地面重赋高度，A* 搜索仍有线性高度参考限制。
- **加速度上限**：RM 的 MPPI 加速度上限为 UNKNOWN，仿真用的是示例值。
- **安全余量**：用户未给出，当前 `safety_margin = 0`。
- 详细限制与未测项见 [测试结果](docs/testing/s3_results.md)。
