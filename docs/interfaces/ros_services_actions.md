# ROS 服务与 Actions

本表区分代码确实创建的业务服务、依赖组件提供的接口，以及 XML 中被实际调用的动作。未运行 introspection；ROS 参数服务不逐项列出。

## Services

| 服务 | type | server → client | 用途/证据/状态 |
|---|---|---|---|
| /trigger_hero_relocation | std_srvs/srv/Trigger | relocation_node → 外部客户端 UNKNOWN | R8 create_service/trigger_super_callback；参数 trigger_hero_service_name；CONFIRMED |
| /armor_detector/set_mode | hnurm_interfaces/srv/SetMode | 当前树 server 已删除，外部是否提供 UNKNOWN → uart_node | UART 保留 client；UART 的 checkAndUpdateMode 调用在接收路径被注释，不声明运行中持续调用 |
| local_costmap/clear_entirely_local_costmap | nav2_msgs/srv/ClearEntireCostmap | Nav2 local_costmap → bt_node 的 ClearEntireCostmap | R13 实际加载 XML 引用；类型/默认服务实现见所装 Nav2 头文件，运行可用性 UNKNOWN |
| 各 Nav2 节点 change_state/get_state 等 | lifecycle_msgs 对应服务 | lifecycle 节点 → lifecycle_manager | R3/R4 明确管理名单；并非所有被创建组件都在名单中 |

`registration/params/default.yaml` 有 decision_reset_topic 字段，但所读 registration_node.cpp 未建立相应订阅；不能据参数把它登记为已实现 reset 接口。历史 launch 注释里的 `/map_save` 也不作为当前导航已存在服务。

## Actions

| action | type | server / client | 当前证据和迁移影响 |
|---|---|---|---|
| compute_path_to_pose | nav2_msgs/action/ComputePathToPose | planner_server / 独立 bt_node + Nav2 BT | decision.cpp 加载插件，simple_test.xml 四种导航子树调用，planner_id=GridBased；替换全局规划时需迁移 |
| follow_path | nav2_msgs/action/FollowPath | controller_server / 独立 bt_node + Nav2 BT | R13 执行，通常 controller_id=FollowPath；SCAN topic 没有相同 cancel/result/feedback 契约 |
| navigate_to_pose | nav2_msgs/action/NavigateToPose | bt_navigator / RViz 或外部 | Nav2 navigator 被装配，决策加载此插件；实际 simple_test.xml 主逻辑直接调用规划/跟踪，不等于经此 action |
| navigate_through_poses | nav2_msgs/action/NavigateThroughPoses | bt_navigator / 外部 UNKNOWN | 配置列出相关插件；本次主决策树未见实际调用 |
| compute_path_through_poses | nav2_msgs/action/ComputePathThroughPoses | planner_server / 备选 BT | 插件被加载，test.xml 有用法；不是默认 simple_test.xml 的主链 |
| spin / backup / drive_on_heading / assisted_teleop / wait | nav2_msgs 对应 action | behavior_server / 恢复 BT 或外部 | R5 behavior_plugins；发速度者也必须纳入命令仲裁 |
| follow_waypoints / smooth_path | nav2_msgs/action/FollowWaypoints、SmoothPath | waypoint_follower / smoother_server | launch 创建但未纳入生命周期列表；活动状态 UNKNOWN |

上述 Nav2 默认 action 名/类型可从本机 `/opt/ros/humble/include/nav2_behavior_tree/.../plugins/action/` 和 nav2_msgs action 定义静态复查；部署版本仍需后续固定。没有把 action 生成的内部 feedback/status topics 误列成用户自定义业务 topics。

## SCAN 的缺失契约

S2/S8 注册的业务接口是 topics/timers，未提供 NavigateToPose、FollowPath 等 action server。`DataDisp` 为调试消息；`execution_frozen` 是控制器对 FSM 的时间冻结反馈，都不能当作任务执行结果。

完全替代 Nav2 至少需要候选任务接口提供：任务 ID、接受/拒绝、路径更新、取消/抢占、成功判定、规划失败/输入失效、反馈、停止确认。实现形式（兼容 Nav2 actions 或新任务接口）**PROPOSED 细节，按已认可的 C 方向规划**。保持消息包接口不变并不意味着保留所有 Nav2 server；具体边界见 [目标架构](../architecture/target_architecture.md)。
