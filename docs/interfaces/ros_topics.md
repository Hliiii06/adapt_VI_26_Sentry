# ROS 话题与消息清单

基线见 [入口](../README.md)，节点责任见两个架构文档。表覆盖导航、定位、感知、底盘和 SCAN 的关键接口；图像检测调试话题不逐项展开。这是静态清单，不宣称每个话题在实车上都有发布者。

记号：`S` = 源码显式 SensorDataQoS；`R(n)` = rclcpp 默认可靠、volatile、KeepLast(n)；`TL` = 显式 transient_local。只有整数 depth 的发布/订阅按默认 rclcpp QoS 解释；依赖包端点未核实处标 UNKNOWN。无命名空间时将相对名按根话题列出。

## RM 传感器、定位和地图

| topic | type | 发布者 → 订阅者 | frame / 时间戳 | QoS、频率、用途与证据 |
|---|---|---|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg | Livox → laserMapping | livox_frame；驱动数据时钟待核实 | 驱动 queue_size 默认可靠；LIO R(200000)；launch 10 Hz；R6、驱动 lddc.cpp |
| /livox/imu | sensor_msgs/msg/Imu | Livox → laserMapping | 驱动 frame；时钟 UNKNOWN | LIO R(200000)，频率 UNKNOWN；R6 subscribe |
| /left_camera/image | sensor_msgs/msg/Image | 相机/republish → LIVO 图像回调 | 相机 frame/time UNKNOWN | 默认 img_en=0；不算默认必需输入；R6/mapping_mid360.launch.py |
| /cloud_registered | sensor_msgs/msg/PointCloud2 | laserMapping → registration、ground_segmentation | camera_init；publish_frame_world 的发布时 now | R(100)→S；世界系去畸变/累计发布云；R6/R8、segmentation_params.yaml |
| /Odometry | nav_msgs/msg/Odometry | laserMapping → TfTransformer、PointCloudNode | camera_init/aft_mapped；now | R(10)→S；主 pose，twist 未填；R6 publish_odometry |
| /LIVO2/imu_propagate | nav_msgs/msg/Odometry | LIVO IMU 传播 → 当前导航未见消费者 | world，child 未填；newest_imu.header.stamp | R(10000)；默认 imu_rate_odom=true；4 ms timer 且受新 IMU/初始化门控；填入世界系 vel_end，是候选速度源，frame/原点仍需适配；R6 imu_prop_callback |
| /Odometry_transformed | nav_msgs/msg/Odometry | TfTransformer → Nav2 controller/bt_navigator | odom/base_footprint；沿用输入 stamp | S；Nav2 订阅 QoS UNKNOWN；R7 odom_callback、R5 odom_topic |
| /initialpose | geometry_msgs/msg/PoseWithCovarianceStamped | RViz/外部 → TfTransformer | 回调未校验/转换 header | 订阅 S；不是已启用 AMCL 输入；R7 |
| /relocalization/state_for_tf | std_msgs/msg/Bool | relocation_node → TfTransformer | 无 header | R(10)→S，373 ms timer；TF 所有权标志，不等于有效定位；R8/R7 |
| /registration_status | std_msgs/msg/String | relocation_node → 外部观察/UNKNOWN | 无 header | R(10)，373 ms；INIT/TRACKING 等状态；R8 timer_callback |
| /global_pcd_map | sensor_msgs/msg/PointCloud2 | relocation_node → 显示/UNKNOWN | map；发布时刻 | R(10)；PCD 地图显示，不是 Nav2 OccupancyGrid；R8 |
| /registration/pointcloud_registered | sensor_msgs/msg/PointCloud2 | relocation_node → 显示/UNKNOWN | 代码标 odom；now | R(10)；当前累积源云输出，名称不能证明已变到 map；R8 配准成功路径 |
| /path | nav_msgs/msg/Path | laserMapping → 显示 | camera_init | R(10)；历史轨迹，不是给 SCAN 的全局参考路径；R6 |
| /segmentation/ground | sensor_msgs/msg/PointCloud2 | ground_segmentation → 显示/UNKNOWN | 复制输入 header | S；地面点；ground_segmentation_node.cc::scanCallback |
| /segmentation/obstacle | sensor_msgs/msg/PointCloud2 | ground_segmentation → PointCloudNode | 复制输入 header，默认 camera_init | S→S；非地面点；同上及 R9 |
| /pointcloud | sensor_msgs/msg/PointCloud2 | PointCloudNode → STVL、pointcloud_to_laserscan | base_footprint，保留输入 stamp | S；按该 stamp 查 TF（0.1 s timeout），过滤半径 0.25 m 和 z∈(-0.35,1.0) m；R9 pointcloud_callback |
| /scan | sensor_msgs/msg/LaserScan | pointcloud_to_laserscan → local_costmap | base_footprint；由输入云派生 | S；有 scan 订阅者才订阅 cloud；launch scan_time=.3333 不等于实测 3 Hz |
| /map | nav_msgs/msg/OccupancyGrid | map_server → global_costmap static_layer | map | 消费端配置 map_subscribe_transient_local=true；发布 QoS 需核依赖；R4/R5 |
| /global_costmap/published_footprint | geometry_msgs/msg/PolygonStamped | global_costmap → PubRobotStatus | global_frame=map | 决策 S；用 footprint 估算全局位置；R14/decision params |
| /local_costmap/costmap_raw、/local_costmap/published_footprint | nav2_msgs/msg/Costmap、geometry_msgs/msg/PolygonStamped | local_costmap → behavior_server | odom | R5 指定；依赖端 QoS/频率实际 UNKNOWN |

Livox launch 设 `xfer_format=4`，驱动扩展支持 CustomMsg 与附加点云发布；不能仅凭 launch 注释中“0/1”判断实际类型。主 LIVO `lidar_type=1` 选择 CustomMsg 订阅。

## RM 命令与任务旁路

| topic | type | 发布者 → 订阅者 | QoS/frame/用途与证据 |
|---|---|---|---|
| /cmd_vel | geometry_msgs/msg/Twist（消费者明确） | composition controller/behavior → UART、可选 PubRobotStatus | UART ServicesQoS（可靠）、决策 S；无 header；Nav2 实际输出类型 UNKNOWN，见下文；R3/R10/R11 |
| /cmd_vel_nav | 依赖 Nav2 实际版本 | 非 composition controller → 本分支缺 smoother；composition smoother 的输入名 | R3；不是两个分支都连通 |
| /cmd_vel_remap | geometry_msgs/msg/Twist | PubRobotStatus → 显式配置该输入的 UART | R(10)→可靠；默认 UART YAML 不用此 topic；R14/R11 |
| /vision_recv_data | hnurm_interfaces/msg/VisionRecvData | UART → TfTransformer、其他视觉/决策消费者 | S；header=serial，stamp=now；R10；决策参数另设 /my_recv_data，存在不一致 |
| /my_recv_data | hnurm_interfaces/msg/VisionRecvData | 发布源 UNKNOWN → 决策配置 | S；不自动等于 /vision_recv_data；decision param/params.yaml |
| /vision_send_data | hnurm_interfaces/msg/VisionSendData | 视觉解算/外部 → UART、PubRobotStatus | S 订阅；目标及云台控制；R10/R14 |
| /decision/vision_send_data | hnurm_interfaces/msg/VisionSendData | 配置期望的决策发布源 UNKNOWN → UART | S 订阅；UART 缓存 gesture；不能由名称推断 bt_node 已发布该消息 |
| /decision/spin_control、/decision/scan_center_angle | std_msgs/msg/Float32 | PubRobotStatus → UART | R(10)→S；控制策略；yaw 单位仍需协议确认 |
| /decision/enable_180_scan | std_msgs/msg/Bool | PubRobotStatus → UART | R(10)→S |
| /decision/back_target_state | std_msgs/msg/Bool | 发布源 UNKNOWN → UART | S；决策有参数但所读 ROS 初始化未见对应 publisher |
| /decision/gesture | hnurm_interfaces/msg/Gesture | PubRobotStatus → 消费者 UNKNOWN | R(10)；不是 /decision/vision_send_data 的自动替代 |
| /decision/is_target_at_home | std_msgs/msg/Bool | PubRobotStatus → UNKNOWN | R(10) |
| /back_target | std_msgs/msg/Float32 | 后视模块/UNKNOWN → UART、PubRobotStatus | S；目标角度/特殊哨兵值；R10/R14 |
| /is_in_special_area | std_msgs/msg/Bool | 发布源 UNKNOWN → UART | S；触发 linear.z 自定义语义 |

`enable_stamped_cmd_vel=true` 出现在 R5；当前仓库没有固定 Nav2 二进制版本证据，不能据此断定发布 TwistStamped，也不能保证仍是 Twist。后续要同时检查依赖版本和实际 endpoint 类型。本轮不运行 `ros2 topic info`。

## SCAN2 核心话题

以下名称是节点相对名；S1 的真机/仿真映射见 SCAN 架构。

| topic | type | 发布者 → 订阅者 | frame/time | QoS/频率/证据 |
|---|---|---|---|---|
| body_pose | nav_msgs/msg/Odometry | 外部 LIO 或模拟器 → FSM、GridMap、closed loop | 数值须统一规划系；twist 被当成该系速度 | 订阅 S；S2/S4/S8 |
| sensor_pose | nav_msgs/msg/Odometry | 外部传感器 pose/renderer → GridMap | 规划系下射线原点与姿态 | 订阅 S；lidar 缓存最新 pose；S4 sensorPoseCallback |
| cloud | sensor_msgs/msg/PointCloud2 | 外部云/renderer → GridMap | world 或 sensor，由 cloud_is_world 决定；header 不驱动 TF 转换 | S；S4 cloudCallback |
| depth | sensor_msgs/msg/Image | 相机/renderer → GridMap | 相机光学投影；支持深度转换 | S，ApproximateTime 同步 sensor_pose；S4 |
| move_base_simple/goal | geometry_msgs/msg/PoseStamped | RViz/上层 → FSM Mode 1 | XY 直接使用；z 取初始 body height；不做 frame 转换 | R(1) 订阅；S2 rvizGoalCallback/waypointCallback |
| initial_path | nav_msgs/msg/Path | 路线提供者/演示发布器 → FSM Mode 3 | 地面路线 z，加 body_height；不按 header 转换 | FSM R(1)、volatile；演示发布器可靠 TL(1)，等待 odom 和订阅者后单次发布；没有 odom 时 FSM 会丢弃输入 |
| planning/bspline | scan_planner_msgs/msg/Bspline | FSM → closed/open loop | **无 frame header**；有 start_time | R(10)；事件/重规划驱动，非固定频率；S2/S8 |
| planning/go2_execution_frozen | std_msgs/msg/Bool | closed loop → FSM | 无 header | R(10)，控制 timer 100 Hz；yaw 对齐时冻结 |
| cmd_vel | geometry_msgs/msg/Twist | closed loop → 模拟器或外部控制接口 | body XY + yaw rate，无 header | R(20)，100 Hz；真机 remap 到 /cmd_vel；S8 |
| planning/data_display | scan_planner_msgs/msg/DataDisp | FSM → 显示/UNKNOWN | Header + a–e 调试字段 | R(100)；不是任务完成/失败反馈 |
| grid_map/occupancy、occupancy_inflate、unknown、depth_cloud（均以 grid_map/ 为前缀） | sensor_msgs/msg/PointCloud2 | GridMap → 可视化 | grid_map.frame_id，默认 world | S；20 Hz 可视化 timer，各输出受条件控制；S4 |
| grid_map/sliding_map_bbox | visualization_msgs/msg/Marker | GridMap → RViz | grid_map.frame_id | R(10)；S4 |
| grid_map/sensor_pose_extrinsic | nav_msgs/msg/Odometry | depth 回调 → 显示/UNKNOWN | 复制 pose header，child 加后缀 | R(10)；不是 lidar 分支必发输出；S4 |
| self_inflation | visualization_msgs/msg/Marker | FSM → RViz | self_inflation_frame_id，来自 grid_map.frame_id | 可靠 TL(1)；body pose 回调驱动；S2 |
| goal_point、global_list、init_list、optimal_list、a_star_list | visualization_msgs/msg/Marker | PlanningVisualization → RViz | 核心显示 world，部分辅助函数 map 硬编码 | 可靠 TL(20)；traj_utils/src/planning_visualization.cpp；不能当成 Path 消息 |

更新已移除 PointCloudNode 的 `/special_areas` 订阅和 `/transformed_special_area` 发布；YAML 残留 filter_topic 不构成端点。`/path` 发布时同步更新 header.stamp。

## 自定义消息边界

RM `hnurm_interfaces/msg/decision/` 提供 Area、SpecialArea、Path、AllPaths、ZoneEndPoint2D 等；`msg/uart/` 提供 VisionSendData/RecvData、Gesture 等；`msg/vision/` 提供目标/调试/ChassisCmd 等。**hnurm_interfaces/msg/Path 不等于 nav_msgs/msg/Path**，也不等于 SCAN Bspline。

`hnurm_interfaces/new_msg/{NavToVision,VisionToNav,TargetType,TargetState}.msg` 是新增草稿；当前 CMake 只匹配 msg/*/*.msg，未生成这些类型，未发现已接入导航/串口的证据。C 首期不依赖它们。

SCAN2 自定义消息独立在 `planner/scan_planner_msgs/msg/`。ROS1 的消息原在 scan_planner 包，不能直接跨 ROS 版本复用类型。初次适配应保留各自消息，通过明确转换连接。

## 本适配新增的话题（全向哨兵）

以下话题都在 `/sentry_sim` 命名空间内，是仿真适配层的一部分，**不属于 RM 原有接口**。

| topic | type | publisher | subscriber | QoS | frame | 时间戳来源 | 频率 |
|---|---|---|---|---|---|---|---|
| `planning/reset` | `std_msgs/msg/Bool` | 测试记录器 / 操作者 | `scan_replan_fsm`、`closed_loop_controller` | reliable、volatile | 无 | — | 事件触发 |
| `planning/task_active` | `scan_planner_msgs/msg/TaskAuthorization` | `scan_replan_fsm` | `closed_loop_controller` | **reliable + transient_local**，depth 1 | `world`（仅填 header，不用于变换） | `node->now()`（系统时钟，`use_sim_time=false`） | 事件触发（接受新任务 / 取消） |
| `ground_surface` | `sensor_msgs/msg/PointCloud2` | `map_pub` | RViz | reliable + transient_local | `world` | 发布时 `now()` | 与 `publish_rate` 同 |
| `terrain_surface_mesh` | `visualization_msgs/msg/Marker`（TRIANGLE_LIST） | `map_pub` | RViz | reliable + transient_local | `world` | 发布时 `now()` | 与 `publish_rate` 同 |
| `body_marker` | `visualization_msgs/msg/Marker`（CYLINDER） | `go2_kinematic_sim` | RViz | reliable + transient_local | `world` | 仿真步时间 | 与仿真步同 |

### `scan_planner_msgs/msg/TaskAuthorization`

```
std_msgs/Header header
bool active
uint32 task_id
```

**为什么不是 `std_msgs/msg/Bool`**：取消时执行端会**立即本地锁止**（不等规划端），
但可能有一条**在取消之前发布、延迟到达**的授权消息。只有 bool 无法区分它与新任务的授权，
会把已锁止的执行端重新放行。`task_id` 由规划端每接受一个新任务自增，
执行端只接受 `task_id > 已撤销到的编号` 的授权。

**语义**：
- `active=true`：规划端接受了一个新任务，允许执行。
- `active=false`：撤销授权，执行端应立即停车并遗忘轨迹。
- `task_id`：单调自增；撤销消息携带当前编号，执行端据此记录"已撤销到哪一号"。

**验证**：`scripts/scenario.sh cancel_race` 注入 `task_id=1` 的延迟授权，
日志出现 `Ignoring stale task authorization task_id=1 (revoked up to 3)`。

### 高度相关话题的边界（重要）

`ground_surface` / `terrain_surface_mesh` / `body_marker` **仅用于 RViz 显示**。
在贴地支撑面实验中，地面点不能当成机体障碍进入占据栅格；
下述无支撑面预览则刻意保留原始地面，以离地圆柱检查三维净空，不代表贴地通行。
地形高度通过地面高度**网格文件**（`ground_grid_file`）提供给运动模拟器与规划器，
不通过话题。详见 [地形与高度跟随](../testing/terrain_following.md)。

### 2026-10-02 模拟点云几何修复

`pcl_render_node` 增加参数 `preserve_map_geometry`（节点默认 false，sentry launch 默认 true），
静态返回改用选中的降采样地图点 XYZ，避免角度格反算虚增洞顶厚度。
没有新增/改名 topic，没有改变 type、publisher/subscriber、QoS、frame、时间戳来源、频率或 TF。
变化是模拟点云的几何内容，不是实车感知协议；可见性仍近似。
地形支撑层继续通过离线网格文件提供。见 [本轮证据及限制](../testing/pcd_route_fix.md)。

### 2026-10-06 可选 waypoint_z_preview（仅 Mode 2 诊断）

默认闭环接口不变。此可选模式不启动 `closed_loop_controller`、`go2kinematicsim`，
只由已有 `open_loop_controller` 生成模拟 odom，无实车连接。

| 话题 | 类型 | 发布 → 订阅 | QoS | frame/时间戳/频率 |
|---|---|---|---|---|
| `/sentry_sim/body_pose` | `nav_msgs/msg/Odometry` | open_loop_controller → SCAN FSM/GridMap、雷达渲染、记录器 | 发布 Reliable/Volatile/KeepLast 20；订阅保持原样 | world → base；节点系统时钟 now；100 Hz；twist 为 world 系导数 |
| `/sentry_sim/planning/bspline` | `scan_planner_msgs/msg/Bspline` | SCAN → open_loop_controller（取代闭环跟踪器） | Reliable/Volatile/KeepLast 10 | world 坐标控制点；规划器 start_time；成功规划事件触发 |

本模式无 `cmd_vel` 发布者；积分器的 `body_marker`、`path` 也不发布。
规划器原有 `self_inflation` 包络显示不变。TF 名称、发布器与频率不变，
仅其依赖的 odom 位姿源从速度积分改为样条求值。
开环执行器不消费 `planning/task_active` 授权或 `planning/reset` 取消；终止实验用 Ctrl-C。
详见[模式边界、六航点与证据](../testing/mode2_waypoint_z_preview.md)。


## 影子接入话题（I1/I2，已实现）

`/sentry_scan` 命名空间是**实车影子入口**（真实 RM 输入、无底盘输出）。
契约、frame/时间/QoS 与降级规则以[影子输入契约](shadow_input_contract.md)为唯一来源；
下表只登记端点，避免出现第二份会漂移的说明。

| topic | type | 发布者 → 订阅者 | QoS | frame / 时间戳 |
|---|---|---|---|---|
| `/sentry_scan/body_pose` | `nav_msgs/msg/Odometry` | `rm_input_adapter` → `scan_planner_node`(FSM/GridMap)、`closed_loop_controller`、`shadow_guard` | SensorDataQoS | 规划系（默认 `odom`）；来源 stamp；`twist` 为规划系机体参考点速度 |
| `/sentry_scan/sensor_pose` | `nav_msgs/msg/Odometry` | `rm_input_adapter` → GridMap | SensorDataQoS | 规划系射线原点；与云同一 stamp |
| `/sentry_scan/cloud` | `sensor_msgs/msg/PointCloud2` | `rm_input_adapter` → GridMap | SensorDataQoS | 已变换到规划系；保留来源 stamp |
| `/sentry_scan/health` | `diagnostic_msgs/msg/DiagnosticArray` | `rm_input_adapter` → 观测/记录 | reliable、volatile、depth 10 | — |
| `/sentry_scan/health_ok` | `std_msgs/msg/Bool` | `rm_input_adapter` → `shadow_guard` | reliable + transient_local、depth 1 | — |
| `/sentry_scan/planning/reset` | `std_msgs/msg/Bool` | `rm_input_adapter`（失效时）→ FSM、跟踪器 | reliable、volatile、depth 10 | — |
| `/sentry_scan/cmd_vel_candidate` | `geometry_msgs/msg/Twist` | `closed_loop_controller`（`cmd_vel` remap）→ `shadow_guard` | reliable、volatile、depth 20 | 机体系 vx/vy；`angular.z` 恒 0（`yaw_candidate_enabled=false`） |
| `/sentry_scan/cmd_vel_shadow` | `geometry_msgs/msg/Twist` | `shadow_guard` → 记录/显示 | reliable、volatile、depth 1 | 同上；**不接车** |

**影子入口不存在** `/cmd_vel`、`/cmd_vel_remap`、`/sentry_scan/cmd_vel` 发布者；
`shadow_guard` 周期检查这些话题是否被别的节点发布，一旦出现即把影子输出归零。
`/sentry_scan` 入口不启动 `map_pub`、`pcl_render_node`、`go2_kinematic_sim`、`open_loop_controller` 或 UART。

新增参数（`closed_loop_controller`）：`yaw_candidate_enabled`（默认 true，影子置 false，只影响 `angular.z`）；
新增参数（GridMap）：`grid_map.strict_sensor_pairing`（默认 **false**，仿真行为不变）、
`grid_map.sensor_pairing_tolerance`（默认 0.05 s）；
新增参数（`PlanningVisualization`）：`visualization_frame_id`（默认空 = 保持上游 world/map 硬编码）。
