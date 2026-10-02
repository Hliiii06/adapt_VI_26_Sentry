# 架构调研入口

记录日期：2026-10-01。范围：静态源码、launch、配置、消息与构建定义。本轮没有运行 ROS 节点、仿真或实车验证。用户确认当前 RM 配置已实车通过 RViz 2D goal 规划与行驶；该结论来源于用户验证，不是本轮测试。

## 版本基线

| 简写 | 工作区相对路径 | 参考版本 | 状态 |
|---|---|---|---|
| RM | `../VI_26_Sentry` | main，`7dfe71a5dbac8bd9b14cb618f0df960942611156` | 用户已拉取更新；main 与本地 origin/main 一致；本轮只读复核 |
| SCAN2 | `../../SCAN-Planner-Ros2` | main，`103bce48bd9de783511d20e286c5e6299b79e47a` | 干净；本地和已记录远端引用仅 main，未发现 ros2-community |
| SCAN1 | `../../SCAN-Planner` | main，`f12161392264e57590ae4aa00c24208c9e3b2a82` + 用户修改 | 只用于注释和实现理解，不作为 ROS 2 接口依据 |

RM 保留未跟踪的 `src/Sophus/`、`src/hnurm_navigation/BRINGUP_LAUNCH_EXPLAINED.md`。SCAN1 保留六个已修改源码：`bspline_optimizer.cpp`、`uniform_bspline.cpp`、`grid_map.h`、`plan_container.hpp`、`planner_manager.cpp`、`scan_replan_fsm.cpp`，以及未跟踪 `AGENTS.md` 和 `PROJECT_CODE_READING_GUIDE.md`。因此其注释不能自动代表 SCAN2 行为。

本文档中的 `RM/src/...`、`SCAN2/src/...` 都相对于上表源码根目录。快照之外的发布版本、依赖实现和实车参数覆盖属于 UNKNOWN。参考仓库中的 `build/`、`install/` 可能来自其他提交，不用它们推断 main 的运行行为。

## 最新实施入口

S0–S3 已在 `src/` 内实施，Codex 第一轮审查发现的 4 项缺陷已修正并补充受控碰撞场景，
等待 **Codex 复审**；尚未接 VI_26_Sentry 实车，RViz 图形交互未验证。

- [S0 基线](migration/s0_baseline.md)：PCD 分析、机器人/外参/限速输入、坐标系与高度约定。
- [实施报告](migration/implementation_report.md)：相对上游 `103bce4` 的改动清单、理由，
  以及第二轮对 Codex 审查意见的处理。
- [S3 结果](testing/s3_results.md)：受控碰撞场景矩阵、三模式与失效停止实测数据、
  独立净空检查、上一轮缺陷与修正、未验证项。
- [Codex 第二轮复审修正](testing/review_round2_fixes.md)：优化后才赋地形高度、
  显示与包络几何一致、取消立即本地锁止 + 授权带 task_id、越界拒绝规划/停在边界、
  停车判据改完整时长；含 cancel_race / goal_out_of_grid / terrain_lateral 三个新场景。
- [真实场地地形核对](testing/field_terrain_check.md)：用户给的三处地形坐标逐处核对，
  **更正了 tunnel_diagnosis.md 前面过于确定的"被坎隔开"结论**；含受控洞口场景
  （高洞通过 / 低洞拒绝）与判据推导（阈值为 0.25 m，不是 0.375 m）。
- [地形与高度跟随](testing/terrain_following.md)：地面分离方法、高度跟随、
  合成坡道与真实场地的实测、局限、RViz 查看命令。**注意：该文更正了 S3 里
  "x=−6 走廊净空 1.187 m"的旧结论（那是删点造成的假象）。**
- [原始证据](testing/evidence/)：场景报告、`cmd_vel` 记录、日志摘要。
- [合成测试地图](testing/maps/)：几何尺寸明确的受控地图与 `INDEX.txt`。
- [实施计划](migration/plan.md)、[harness 交接说明](migration/implementation_handoff.md)：阶段划分与交付边界。

仓库入口与构建/启动命令见根 [README.md](../README.md)。

## 15 分钟阅读顺序

1. [当前导航](architecture/current_navigation.md)：入口、数据流与实际 Nav2 职责。
2. [SCAN](architecture/scan_planner.md)：算法边界、三维能力和控制约束。
3. [接口映射](migration/interface_mapping.md)与[缺口](migration/gap_analysis.md)。
4. [候选架构](architecture/target_architecture.md)与[决策状态](migration/decisions.md)。
5. [计划](migration/plan.md)、[当前进展](migration/progress.md)和[验证计划](testing/validation_plan.md)。

查表入口：[包清单](architecture/package_inventory.md)、[Topics](interfaces/ros_topics.md)、[Services/Actions](interfaces/ros_services_actions.md)、[TF](interfaces/tf_tree.md)、[底盘](interfaces/chassis_interface.md)、[构建说明](testing/build_notes.md)。

## 证据约定

- **CONFIRMED**：在上述版本源码或配置中直接存在；若为用户实车验证则显式注明来源。静态源码本身不是运行成功证明。
- **INFERRED**：由调用链推导；说明成立条件。
- **UNKNOWN**：缺少运行记录、硬件协议或依赖版本等证据。
- **PROPOSED**：设计建议，尚未获得实施批准。

## 关键源码入口

以下链接从本文件出发可直接访问参考源码；子文档使用简写和函数名定位。

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

## 最重要的结论

当前 RM 使用三维感知和定位，但以二维 costmap + Smac2D + MPPI Omni 导航。SCAN2 使用三维占据/碰撞，但这份实现的 z 来自参考高度、A* 插值和初始化，优化器不沿 z 优化；闭环只控制 XY/yaw。这正是用户所指的“三维导航”，不额外要求自由 z 优化、跨层全局搜索或完整轮地接触规划。C 方向已获认可：保留已验证的感知定位/底盘基础，适配 SCAN 与全向执行；仍需补齐输入、任务和停止契约。更新详情见 [基线更新](migration/baseline_update.md)，实施顺序见 [计划](migration/plan.md)。
