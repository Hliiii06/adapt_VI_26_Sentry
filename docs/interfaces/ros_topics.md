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
