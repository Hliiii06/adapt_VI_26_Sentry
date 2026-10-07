# RM → SCAN 接口映射

基线/证据见[文档入口](../README.md)。Compatible 按**实际语义**判断，不按类型同名判断。
本表核对的是**上游 SCAN2 `103bce4` 与 RM main 的差异**；最后一列区分**本仓库已实施**与仍待实现。

> **更正（2026-10-07）**：不要再写“所有 adapter 未实现 / SCAN 没有现成执行端”。
> RM 侧输入与命令 adapter 仍未实现（PROPOSED，I1/I2 范围）；但
> **全向执行端已在本仓库 `src/` 实施**——`closed_loop_controller` 已去掉“先对齐 yaw 才平移”门槛，
> 输出机体系 `vx/vy` + yaw 策略，并有速度闭环仿真证据（见[实施报告](implementation_report.md)）。
> `open_loop_controller` 只用于无支撑面预览诊断，不是实车速度接口。

| SCAN requirement | Current RM source | Type | Compatible | Adapter / 判定依据 | 本仓库实施状态 |
|---|---|---|---|---|---|
| ROS2 消息传输基础 | rclcpp + Humble 工作区 | ROS2 标准消息 | DIRECT | 同为 ROS2；不代表运行依赖已验证 | 已具备 |
| body_pose：同一规划系中的机身 pose | `/Odometry_transformed` | nav_msgs/Odometry | MAJOR_ADAPTATION | TF 组合/参考点需先验证；frame 不可只改名；R7、S2 | **已实现**（影子适配包用消息时刻 TF 组装）；参考点是否等于几何中心仍 UNKNOWN |
| body_pose 中的线速度 | `/LIVO2/imu_propagate`（新默认开启） | nav_msgs/Odometry.twist | MINOR_ADAPTATION（待验证） | 填入 `vel_end` 世界系速度；主 `/Odometry` 仍未填；须核对 world 数值系、IMU/机身参考点和时间；S2/R6 | **已实现但默认降级**：`world` 无 TF 时置零并写诊断，杆臂未补偿；需 I1 核对 |
| sensor_pose：射线原点 | 原 LIO pose + 标定外参 | nav_msgs/Odometry | MINOR_ADAPTATION | 先确认 IMU/LiDAR 原点，再按云采样时刻生成；不能直接用底盘 pose；S4 | **已实现**：按云 stamp 查 TF，与云同戳发布 |
| 世界/规划系点云 | `/cloud_registered` | sensor_msgs/PointCloud2 | MINOR_ADAPTATION | camera_init 到规划系的时刻对齐；`cloud_is_world=true` 时禁用重复外参；保留地形信息 | **已实现**：adapter 变换到规划系，SCAN 侧 `cloud_is_world=true`/`need_extrinsic=false` |
| 已过滤点云 | `/pointcloud` | sensor_msgs/PointCloud2 | MAJOR_ADAPTATION | 现为 base_footprint 且按高度截断；适合作障碍候选，首期优先 `/cloud_registered` | 未实现 |
| TF/规划 frame | map→odom→base_link→base_footprint | tf2 | MAJOR_ADAPTATION | SCAN callback 不自动查 TF、可视化硬编码；需要统一契约和重定位策略 | **影子取 `odom`（PROPOSED，待 I1）**；可视化 frame 已可配置；map→odom 跳变会撤销任务 |
| Mode 1 goal | RViz / 决策目标 | PoseStamped / BT blackboard / Nav2 action | MINOR_ADAPTATION | 单个可视化目标可转换 topic/frame；生产任务需额外 action/状态适配 | 仿真 Mode 1 已实现；RM 目标适配未实现 |
| Mode 3 reference path | Nav2 ComputePathToPose 结果 | nav_msgs/Path | MAJOR_ADAPTATION | C 首期使用已知参考 xyz 路线，地面 z + `body_height` 只加一次；不默认接 Nav2 Path | 仿真 Mode 3 已实现；RM 路线适配未实现 |
| 自由跨层/地形全局搜索 | 当前未纳入目标 | — | OUT_OF_SCOPE | 用户目标是 SCAN 当前实现；不作为首期迁移阻断项 | — |
| robot geometry | local radius .26、global radius .03、实际尺寸 | 参数/模型 | MAJOR_ADAPTATION | 上游双圆柱、方向取切线；需实际尺寸及旋转包络 | **已实施**：`offset=0` 单圆柱（半径 0.26 + `safety_margin`） |
| holonomic motion | MPPI Omni、UART vel_x/vel_y | Twist、自定义帧 | MAJOR_ADAPTATION | 上游要求 yaw 对齐，RM yaw 模式不同 | **已实施**：取消对齐门槛，新增 `yaw_mode`(hold/align/spin) |
| Bspline execution | 上游无现成 SCAN 执行端 | scan_planner_msgs/Bspline | **已适配** | 消息无 frame，契约固定为规划系 | **已实施**：改造既有 `closed_loop_controller` 做真速度闭环 |
| 标准 vx/vy/wz 到底盘 | UART Twist 回调 | Twist → VisionSendData | MAJOR_ADAPTATION | `angular.z` 未使用、`linear.z` 重用、`spin_ctrl` 被覆盖；R10 | 未实现；I3 前只有影子候选速度 |
| 速度/加速度限制 | MPPI + 未确认 active 的 smoother | 参数/命令流 | MAJOR_ADAPTATION | 明确执行层限幅、超时和切换策略；不能只抄 YAML | 仿真已加一阶加速度限制；实车阈值 UNKNOWN |
| map representation | PCD、2D static/STVL/local costmap | PointCloud2/OccupancyGrid | MAJOR_ADAPTATION | SCAN 自建局部 log-odds；旧 costmap 不能直接导入 | 仿真用 PCD + 离线地面/障碍分离 |
| goal cancel/result/recovery | Nav2 actions 与决策 BT | nav2_msgs actions | MISSING | SCAN topics 不提供同等生命周期 | 影子用 `planning/reset` + `TaskAuthorization(task_id)`；两者**无共同任务字段**，跨进程安全仍不完整 |
| timestamps / sensor synchronization | 主 LIO 发 `now()`、高频 odom 用 IMU stamp、filter 按云 stamp 查 TF | Header/time | UNKNOWN | 需实测延迟及消息配对；不能假设全链同步 | 未测（B3 范围） |
| QoS | LIO reliable、感知/TF adapter SensorDataQoS | ROS2 QoS | MINOR_ADAPTATION | 分端核对 reliability/durability/history；SCAN goal/path volatile | 仿真端点已登记；实车端点未核对 |
| command watchdog / firmware safety | UART + 电控 | 自定义协议 | UNKNOWN | UART 回调未见超时归零；固件未提供 | 未实现；I1 记录后另评 |

DIRECT 仅表示对应维度可直接复用，绝不意味着完整管线可直接连接；MINOR_ADAPTATION 也需测试 frame、时间和单位。
I1/I2 的输入契约与拟新增接口以[交接 B](real_robot_handoff.md)为准。
