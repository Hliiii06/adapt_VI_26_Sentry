# RM → SCAN 接口映射

基线/证据见 [入口](../README.md)。Compatible 按**实际语义**判断，不按类型同名判断。所有 adapter 均为 PROPOSED，本轮未实现。

| SCAN requirement | Current RM source | Type | Compatible | Adapter / 判定依据 |
|---|---|---|---|---|
| ROS2 消息传输基础 | rclcpp + Humble 工作区 | ROS2 标准消息 | DIRECT | 同为 ROS2；不代表运行依赖已验证 |
| body_pose：同一规划系中的机身 pose | /Odometry_transformed | nav_msgs/Odometry | MAJOR_ADAPTATION | TF 组合/参考点需先验证；frame 不可只改名；R7、S2 |
| body_pose 中的线速度 | /LIVO2/imu_propagate（新默认开启） | nav_msgs/Odometry.twist | MINOR_ADAPTATION（待验证） | 该消息填入 vel_end 世界系速度；主 /Odometry 仍未填。须核对 world 数值系、IMU/机身参考点和时间；S2/R6 |
| sensor_pose：射线原点 | 原 LIO pose + 标定外参 | nav_msgs/Odometry | MINOR_ADAPTATION | 条件：先确认 IMU/LiDAR 原点，再按云采样时刻生成；不能直接用底盘 pose；S4 |
| 世界/规划系点云 | /cloud_registered | sensor_msgs/PointCloud2 | MINOR_ADAPTATION | camera_init 到规划系的时刻对齐；world 模式禁用重复外参；保留地形信息 |
| 已过滤点云 | /pointcloud | sensor_msgs/PointCloud2 | MAJOR_ADAPTATION | 现为 base_footprint，且按高度截断；适合作障碍候选，不能直接当世界点云，首期优先 /cloud_registered |
| TF/规划 frame | map→odom→base_link→base_footprint | tf2 | MAJOR_ADAPTATION | SCAN callback 不自动查 TF、可视化硬编码；需要统一契约和重定位策略 |
| Mode 1 goal | RViz / 决策目标 | PoseStamped / BT blackboard / Nav2 action | MINOR_ADAPTATION | 单个可视化目标可转换 topic/frame；生产任务需额外 action/状态适配 |
| Mode 3 reference path | Nav2 ComputePathToPose 结果 | nav_msgs/Path | MAJOR_ADAPTATION | C 首期使用已知参考 xyz 路线，明确地面 z + body_height；不默认接 Nav2 Path，也不重复加高度 |
| 自由跨层/地形全局搜索 | 当前未纳入目标 | — | OUT_OF_SCOPE | 用户目标是 SCAN 当前实现；不作为首期迁移阻断项 |
| robot geometry | local radius .26、global radius .03、实际尺寸 | 参数/模型 | MAJOR_ADAPTATION | SCAN 双圆柱、方向取切线；需实际尺寸及旋转包络，不照抄 radius |
| holonomic motion | MPPI Omni、UART vel_x/vel_y | Twist、自定义帧 | MAJOR_ADAPTATION | SCAN 有 vy，但要求 yaw 对齐；RM yaw 模式不同 |
| Bspline execution | 无现成 SCAN 执行端 | scan_planner_msgs/Bspline | MISSING | 需轨迹跟踪器或评审改造原 closed loop；消息无 frame |
| 标准 vx/vy/wz 到底盘 | UART Twist 回调 | Twist → VisionSendData | MAJOR_ADAPTATION | angular.z 未使用、linear.z 重用、spin_ctrl 被覆盖；R10 |
| 速度/加速度限制 | MPPI + 未确认 active 的 smoother | 参数/命令流 | MAJOR_ADAPTATION | 明确执行层限幅、超时和切换策略；不能只抄 YAML |
| map representation | PCD、2D static/STVL/local costmap | PointCloud2/OccupancyGrid | MAJOR_ADAPTATION | SCAN 自建局部 log-odds；旧 costmap 不能直接导入为其地图对象 |
| goal cancel/result/recovery | Nav2 actions 与决策 BT | nav2_msgs actions | MISSING | SCAN topics 不提供同等生命周期 |
| timestamps / sensor synchronization | 主 LIO 发布时 now、高频 odom 用 IMU stamp、filter 按云 stamp 查 TF | Header/time | UNKNOWN | 需实测延迟及消息配对；不同输入时间来源不一致，不能假设全链同步 |
| QoS | LIO reliable、感知/TF adapter SensorDataQoS | ROS2 QoS | MINOR_ADAPTATION | 分端核对 reliability/durability/history；SCAN goal/path volatile，不能假设后订阅可重放 |
| command watchdog / firmware safety | UART + 电控 | 自定义协议 | UNKNOWN | UART 回调未见超时归零；固件未提供 |

DIRECT 仅表示对应维度可直接复用，绝不意味着完整管线可直接连接。MINOR_ADAPTATION 也需测试 frame、时间和单位。
