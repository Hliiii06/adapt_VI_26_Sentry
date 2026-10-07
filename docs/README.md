# 文档入口

这是 ROS 2 全向哨兵从 Nav2 向 SCAN-Planner 迁移的文档与实施工作区。
**每个主题只有一个当前入口**；历史过程与证据在 [docs/archive/](archive/README.md)。

## 当前阶段（2026-10-07）

- **交接 A 仓库整理已完成**：映射、保留边界与验证见[整理报告](migration/cleanup_report.md)。
- **下一开发任务未开始**：[实车影子接入交接 B](migration/real_robot_handoff.md)，先做 I1/I2
  影子运行，不驱动车辆；是否开始由用户批准（用户要求完成任务一后暂停）。
- **未接实车、未授权底盘输出**；本仓库输出都在 `/sentry_sim` 命名空间内。
- 参考仓库 RM `../VI_26_Sentry`、SCAN2 `../../SCAN-Planner-Ros2`、SCAN1 `../../SCAN-Planner` 保持只读。

## 按职责找入口

| 主题 | 当前入口 | 说明 |
|---|---|---|
| 工作区规则 | [AGENTS.md](../AGENTS.md) | 边界、参考路径、工程/安全规则 |
| 构建与启动 | 根 [README.md](../README.md)、[scripts/README.md](../scripts/README.md) | 脚本分组与常用命令 |
| 当前任务 | [交接 B](migration/real_robot_handoff.md) | I1/I2 契约、接口、测试矩阵（PROPOSED） |
| 阶段与门槛 | [实施计划](migration/plan.md) | S/I 阶段、执行与停止规则、回退 |
| 进展 | [进展](migration/progress.md) | 当前状态与最近变更（历史见[归档](archive/progress_history.md)） |
| 决策 | [决策记录](migration/decisions.md) | 状态、日期、被谁取代 |
| 接口映射 | [RM → SCAN 接口映射](migration/interface_mapping.md) | 当前差距与已有执行端 |
| 实施改动 | [实施报告](migration/implementation_report.md) | 相对上游 `103bce4` 的改动清单（含历史轮次） |
| RM 当前架构 | [当前 RM 导航](architecture/current_navigation.md) | main `7dfe71a`；用户已实车验证 |
| SCAN 架构 | [SCAN-Planner ROS 2](architecture/scan_planner.md) | 上游基线 + 本仓库实施差异 + 包络参数语义 |
| 目标架构 | [C 方案目标架构](architecture/target_architecture.md) | 分层与未实现的实车边界 |
| 包与依赖 | [包清单](architecture/package_inventory.md) | 两工程 ROS package 清单 |
| 接口契约 | [Topics](interfaces/ros_topics.md)、[Services/Actions](interfaces/ros_services_actions.md)、[TF](interfaces/tf_tree.md)、[底盘](interfaces/chassis_interface.md) | 当前契约与候选实车契约分开标识 |
| 判据与验证 | [验证计划](testing/validation_plan.md) | 判据、门槛、可复用测试 |
| 指定地形实验 | [真实 PCD 路线修复](testing/pcd_route_fix.md) | 三条通道当前操作与限制 |
| 无支撑面预览 | [Mode 2 六 XYZ 航点](testing/mode2_waypoint_z_preview.md) | 与速度闭环严格分开 |
| 地形与高度跟随 | [当前实现](testing/terrain_following.md) | 地面/障碍分离、链路、RViz 图层、局限 |
| 构建/依赖查询 | [构建说明](testing/build_notes.md) | 历史构建尝试与依赖登记 |
| 历史资料 | [归档索引](archive/README.md) | 被取代的报告、失败记录、旧交接跳转 |
| 原始证据 | [evidence/](testing/evidence/) | 场景报告、`cmd_vel` 记录、日志摘要、基线快照 |
| 测试地图 | [maps/](testing/maps/) | 几何尺寸明确的受控地图与航点（运行资产，不搬迁） |

## 版本基线

| 简写 | 工作区相对路径 | 参考版本 | 状态 |
|---|---|---|---|
| RM | `../VI_26_Sentry` | main，`7dfe71a5dbac8bd9b14cb618f0df960942611156` | 用户已拉取；main 与本地 origin/main 一致；只读复核 |
| SCAN2 | `../../SCAN-Planner-Ros2` | main，`103bce48bd9de783511d20e286c5e6299b79e47a` | 干净；仅 main，无 ros2-community 分支 |
| SCAN1 | `../../SCAN-Planner` | main，`f12161392264e57590ae4aa00c24208c9e3b2a82` + 用户修改 | 仅注释与实现理解，不作为 ROS 2 接口依据 |

RM 保留未跟踪的 `src/Sophus/`、`src/hnurm_navigation/BRINGUP_LAUNCH_EXPLAINED.md`；
SCAN1 保留六个已修改源码与未跟踪 `AGENTS.md`、`PROJECT_CODE_READING_GUIDE.md`，
其注释不能自动代表 SCAN2 行为。参考仓库的 `build/`、`install/` 可能来自其他提交，不用于推断 main 行为。
整理前的完整快照见 [cleanup_baseline_2026-10-07.txt](testing/evidence/cleanup_baseline_2026-10-07.txt)。

## 证据约定

- **CONFIRMED**：在给定版本源码/配置中直接存在；若为用户实车验证则显式注明来源。
- **INFERRED**：由调用链或已有数据推导，说明成立条件。
- **UNKNOWN**：缺少运行记录、硬件协议或依赖版本等证据。
- **PROPOSED**：设计建议，尚未获得实施批准。
- 静态源码不是运行成功证明；未由本轮运行验证的内容不能声称本轮实测。

## 关键源码入口

从本文件出发可直接访问参考源码；子文档使用简写和函数名定位（证据编号 R/S）。

| ID | 文件 | 阅读重点 |
|---|---|---|
| R1 | [relocal_nav.sh](../../VI_26_Sentry/relocal_nav.sh) | 多终端启动主链，决策启动被注释 |
| R2 | [Nav2 bringup](../../VI_26_Sentry/src/hnurm_navigation/launch/bringup_launch.py) | 默认 map、params、composition |
| R3 | [navigation_launch.py](../../VI_26_Sentry/src/hnurm_navigation/launch/navigation_launch.py) | 节点、生命周期列表、速度 remap 分支 |
| R4 | [localization_launch.py](../../VI_26_Sentry/src/hnurm_navigation/launch/localization_launch.py) | map_server；AMCL 被注释 |
| R5 | [nav2_params.yaml](../../VI_26_Sentry/src/hnurm_navigation/params/nav2_params.yaml) | MPPI Omni、Smac2D、costmap |
| R6 | [LIVMapper.cpp](../../VI_26_Sentry/src/hnurm_fastlivo2/FAST-LIVO2/src/LIVMapper.cpp) | 输入、点云和 odometry 发布 |
| R7 | [TF transformer](../../VI_26_Sentry/src/hnurm_bringup/src/tf_transformer_node.cpp) | frame 桥接、里程计转换、map→odom 交接 |
| R8 | [registration_node.cpp](../../VI_26_Sentry/src/hnurm_perception/hnurm_registration/src/registration/src/registration_node.cpp) | Quatro、小 GICP、状态与 TF |
| R9 | [pointcloud_filter_node.cpp](../../VI_26_Sentry/src/hnurm_perception/pointcloud_filter/src/pointcloud_filter_node.cpp) | 自身半径/高度过滤、消息时刻 TF、base_footprint 输出 |
| R10 | [uart_node.cpp](../../VI_26_Sentry/src/hnurm_uart/src/uart_node.cpp) | Twist 到串口字段 |
| R11 | [UART default.yaml](../../VI_26_Sentry/src/hnurm_uart/params/default.yaml) | 实际 twist_topic 为 /cmd_vel |
| R12 | [decision.cpp](../../VI_26_Sentry/src/RMUL_Decision/src/hnurm_rmul_decision/src/decision.cpp) | 自有 BT 调用 Nav2 插件 |
| R13 | [simple_test.xml](../../VI_26_Sentry/src/RMUL_Decision/src/hnurm_rmul_decision/param/simple_test.xml) | 实际加载的导航行为树 |
| R14 | [PubRobotStatus.cpp](../../VI_26_Sentry/src/RMUL_Decision/src/hnurm_rmul_decision/plugins/action/PubRobotStatus.cpp) | 目标、裁判、速度与扫描控制 |
| S1 | [run.launch.py](../../../SCAN-Planner-Ros2/src/planner/plan_manage/launch/run.launch.py) | 真机/仿真/控制器分支 |
| S2 | [scan_replan_fsm.cpp](../../../SCAN-Planner-Ros2/src/planner/plan_manage/src/scan_replan_fsm.cpp) | 输入、状态机、碰撞检查、输出 |
| S3 | [planner_manager.cpp](../../../SCAN-Planner-Ros2/src/planner/plan_manage/src/planner_manager.cpp) | 参考多项式、局部 z 插值、轨迹可行性 |
| S4 | [grid_map.cpp](../../../SCAN-Planner-Ros2/src/planner/plan_env/src/grid_map.cpp) | 占据、膨胀、滑窗、传感器输入 |
| S5 | [grid_map.h](../../../SCAN-Planner-Ros2/src/planner/plan_env/include/plan_env/grid_map.h) | 双圆柱碰撞查询和越界语义 |
| S6 | [dyn_a_star.cpp](../../../SCAN-Planner-Ros2/src/planner/path_searching/src/dyn_a_star.cpp) | XY 邻域与插值 z |
| S7 | [bspline_optimizer.cpp](../../../SCAN-Planner-Ros2/src/planner/bspline_opt/src/bspline_optimizer.cpp) | LBFGS、碰撞方向、z 梯度置零 |
| S8 | [closed_loop_controller.cpp](../../../SCAN-Planner-Ros2/src/planner/plan_manage/src/closed_loop_controller.cpp) | XY 速度、yaw 对齐、冻结执行时间 |
| S9 | [planner.yaml](../../../SCAN-Planner-Ros2/src/planner/plan_manage/config/planner.yaml) | 网格、几何体、规划速度参数 |
| S10 | [controllers.yaml](../../../SCAN-Planner-Ros2/src/planner/plan_manage/config/controllers.yaml) | 限速及仿真配置 |

本仓库实施副本 `src/` 的来源与改动边界见[实施报告](migration/implementation_report.md)。

## 最重要的结论

当前 RM 使用三维感知和定位，但以二维 costmap + Smac2D + MPPI Omni 导航。SCAN2 使用三维占据/碰撞，
但这份实现的 z 来自参考高度、A* 插值和初始化，优化器不沿 z 优化；闭环只控制 XY/yaw。
这正是用户所指的“三维导航”，不额外要求自由 z 优化、跨层全局搜索或完整轮地接触规划。
C 方向已获认可：保留已验证的感知定位/底盘基础，适配 SCAN 与全向执行；仍需补齐输入、任务和停止契约。
RM 的 `82d0741→7dfe71a` 变更详情见[归档](archive/baseline_update.md)。
