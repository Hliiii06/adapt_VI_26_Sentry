# 影子接入输入契约（RM → SCAN，I1/I2）

状态：**v1 已实现并在合成输入下通过契约测试**；标 **UNKNOWN** 的项必须由 I1 用现车/录包核对后
才能作为实车依据。本页是 `/sentry_scan/*` 接口的唯一事实入口；实现见
`src/sentry_scan_adapter/`，入口见 [影子 launch](../../src/sentry_scan_adapter/launch/sentry_scan_shadow.launch.py)，
参数模板见 [shadow_contract.yaml](../../src/sentry_scan_adapter/config/shadow_contract.yaml)。

**本页不授权任何底盘输出**：影子入口没有"下发真实命令"的开关，不启动 UART/Nav2/LIO/registration/
机器人/运动模拟器。I3 驱车另行授权（见[交接 B](../migration/real_robot_handoff.md)）。

## 零点五、任务/路线坐标契约（P1 修正）

SCAN 的 FSM **不按 `header.frame_id` 做任何变换**，把收到的数值直接当规划系坐标。因此影子入口
在 FSM 之前插入 `task_adapter`，目标与路线都必须经过它：

| 方向 | 话题 | 类型 / QoS | frame 与时间 |
|---|---|---|---|
| 输入 | `task/goal_in` | PoseStamped / reliable·volatile·1 | **任意 frame**；按消息 stamp 转换 |
| 输出 | `goal` | PoseStamped / reliable·volatile·1 | 规划系；保留输入 stamp，接 FSM Mode 1 |
| 输入 | `task/path_in` | Path / reliable·transient_local·1 | **任意 frame**；整条路线用消息 stamp 的单一 TF |
| 输出 | `initial_path` | Path / reliable·transient_local·1 | 规划系，地面 z；`body_height` 由 SCAN 加一次 |

拒绝规则（**明确拒绝胜过猜坐标**）：空 `header.frame_id`（除非显式 `empty_frame_is_planning=true`）、
查不到 `T_planning<-source`、stamp 缺失/未来超容差/过旧、空路线。拒绝只告警、不发布。
查询与 `rm_input_adapter` 同一时间策略（非阻塞 + `tf_future_tolerance` 顶替）。

Mode 2 的航点来自参数文件，是**配置期输入**，按约定必须是规划系坐标；运行期不做变换（launch 注释已写明）。

## 一、坐标系与参考点（PROPOSED，待 I1 核对）

| 名称 | 参数 | 默认 | 含义与风险 |
|---|---|---|---|
| 规划系 | `planning_frame` | `odom` | 局部连续系；`/Odometry_transformed` 的父 frame。SCAN `grid_map.frame_id` 同步设为它 |
| 任务系 | `task_frame` | `map` | 路线/任务系；仅用于 map→odom 跳变监测 |
| 机体参考 frame | `body_frame` | `base_link` | **UNKNOWN**：RM 的 `base_link` 是否等于几何中心/旋转轴。默认不加偏移，等于把 LIO 机体系当作规划参考点 |
| 参考中心偏移 | `body_center_offset_xyz` | 空 | body 系下 body_frame→参考中心的杆臂；默认不加 |
| 雷达射线原点 | `sensor_frame` | `lidar_link` | `base_footprint→lidar_link` 静态外参来自 `lidar_to_base` |
| 速度源 frame | `velocity_frame` / `velocity_frame_alias` | `world` / 空 | `/LIVO2/imu_propagate` 的 header 写 `world`，但 TF 里通常没有该 frame |

变换一律在**消息时间戳**上查询：

```text
body_pose   = T_planning<-body_frame(stamp)  (+ R(orientation)·body_center_offset)
sensor_pose = T_planning<-sensor_frame(cloud_stamp)     # 与云同一时间戳，保证配对
cloud       = T_planning<-cloud.header.frame_id(cloud_stamp) · cloud
```

消息与 TF 由同一节点在同一回调里发出时，TF 可能尚未进入 buffer（实测差 ~0.2 ms）。
因此查询为**非阻塞**（`tf_lookup_timeout=0`），失败时允许用最新可用 TF 顶替，
但偏差必须 ≤ `tf_future_tolerance`（默认 0.05 s）；请求时刻早于 buffer 起点则直接拒绝。
把 timeout 设成 >0 会在单线程 executor 里阻塞 TF 订阅，使 buffer 越来越旧（实测滞后 >1 s），不要这样做。

## 二、输入话题（只读，RM 侧）

| 输入 | 类型 / 默认话题 | frame、时间戳 | 适配处理 |
|---|---|---|---|
| 机身位姿 | `nav_msgs/Odometry` · `/Odometry_transformed` | 父 `odom`；沿用输入 stamp | 只用作**时间与触发**；位姿实际取自该 stamp 的 TF，避免依赖消息里被解释/改名过的坐标 |
| 速度候选 | `nav_msgs/Odometry` · `/LIVO2/imu_propagate` | header `world`；IMU stamp | 世界系线速度按 TF 旋到规划系；杆臂/角速度缺失时**显式降级**（见第四节） |
| 点云 | `sensor_msgs/PointCloud2` · `/cloud_registered` | `camera_init`；发布时刻 | 按同一 stamp 的 TF 变换到规划系；保留原 stamp；不做"硬件同步"的宣称 |
| TF | `tf2` / `tf_static` | 按消息时刻查询 | `odom→base_link`、`base_footprint→lidar_link`、`odom→camera_init`、`map→odom` |

未使用的输入（保持只读、不接线）：`/segmentation/obstacle`（后续可作对照）、`/pointcloud`
（base_footprint + 高度截断，首期不用）。

## 三、输出话题（`/sentry_scan`）

| 输出 | 类型 / 流向 | QoS | frame / 时间戳 | 频率 |
|---|---|---|---|---|
| `body_pose` | `Odometry`：adapter → FSM/GridMap/跟踪器 | SensorDataQoS | 规划系；来源 stamp；twist.linear 为**规划系**机体参考点线速度 | 随有效 odom |
| `sensor_pose` | `Odometry`：adapter → GridMap | SensorDataQoS | 规划系射线原点；**与云同一 stamp** | 随配对云 |
| `cloud` | `PointCloud2`：adapter → GridMap | SensorDataQoS | 已变换到规划系；保留来源 stamp | 随有效云 |
| `health` | `DiagnosticArray`：adapter → 观测 | reliable/volatile/depth 10 | 无变换语义 | `health_period`（默认 2 Hz） |
| `health_ok` | `Bool`：adapter → shadow_guard | reliable + **transient_local**, depth 1 | 无 | `health_period` |
| `planning/reset` | `Bool`：adapter → FSM + 跟踪器 | reliable/volatile/depth 10 | 无 | 失效时按 `reset_repeat_period` 重发 |
| `cmd_vel_candidate` | `Twist`：跟踪器（remap）→ shadow_guard | reliable/volatile/depth 20 | 机体系 vx/vy；`angular.z` 恒 0（`yaw_candidate_enabled=false`） | 100 Hz 目标 |
| `cmd_vel_shadow` | `Twist`：shadow_guard → 记录/显示 | reliable/volatile/depth 1 | 同上；**只记录，不接车** | `output_rate`（默认 20 Hz） |
| `move_base_simple/goal` | `PoseStamped`：RViz/测试 → FSM Mode 1 | 订阅 depth 1 | 规划系 | 事件 |
| `initial_path` | `Path`：路线发布器 → FSM Mode 3 | 订阅 depth 1 | 规划系地面 z；`body_height` 由 SCAN 加一次 | 事件 |
| `planning/bspline`、`planning/task_active` | `scan_planner_msgs` | 见[话题清单](ros_topics.md) | 规划系；消息本身无 frame | 事件 |

SCAN 侧关键参数：`cloud_is_world=true`、`need_extrinsic=false`（变换与真实外参已在 adapter/TF 完成，
不能再叠一次）、`strict_sensor_pairing=true`、`sensor_pairing_tolerance=0.02`、
`double_cylinder_offset=0`、`double_cylinder_radius=0.26`、`safety_margin=0`、
z 膨胀 = `robot_height/2`（默认 0.125）。

## 四、速度语义与降级（必须显式，不得静默）

`/LIVO2/imu_propagate` 给的是 **IMU 点的世界系速度**，不是机体中心速度。适配层按以下顺序处理，
每个输出在 `health` 里带 `odom.velocity` 字段说明取值来源：

| 情况 | 行为 | `odom.velocity` |
|---|---|---|
| 速度源 frame 有 TF 或 `velocity_frame_alias` 声明等价 | 旋转到规划系 | — |
| frame 无 TF 且未声明别名 | **线/角速度置零** + 限频告警 | `velocity_frame_unresolved(world)` |
| 未配置 `imu_to_body_offset_xyz` | 按点速度使用（启机告警一次）；`require_center_velocity=true` 时改为置零 | `lever_arm_uncompensated` |
| 配置了杆臂 | `v_center = v_point + ω × R_body·offset` | `center_velocity_compensated` |
| 角速度全零且 `require_angular_velocity=true` | 置零 | `angular_velocity_missing` |
| 速度 stamp 超龄/接收超龄 | 置零 | `velocity_stale_stamp` / `velocity_stale_receive` |

**未解决**：RM 是否填充 `twist.angular`、IMU 原点与机体中心的实际杆臂均 **UNKNOWN**；
`velocity_frame_alias` 只有在 I1 核对 `world` 与 `odom` 的数值关系后才能填写。
在核对之前，实车的速度前馈质量不作保证（置零是保守行为，不影响位姿/云驱动的规划）。

## 五、健康门控、失效与恢复

必需输入：`odom`（TF 位置）、`cloud`、`tf`。判定在 `health` 诊断里逐项给出计数/拒绝数/frame/年龄：

- 来源年龄 `now − header.stamp ≤ max_source_age`（默认 0.5 s）：**旧 stamp 反复重发不会绕过**；
- 接收年龄 `now − 本机收到时刻 ≤ max_receive_age`（默认 0.5 s）：输入停发即失效；
- **未来时间戳** `stamp − now > max_future_stamp`（默认 0.05 s）→ 拒绝且**不写入历史**，
  避免一个坏 stamp 让之后所有正常消息被误判成"时间倒退"；
- 时间戳不得倒退（`max_stamp_regression=0`）；有限值/四元数模长检查；空云拒绝；
- **有效点检查（按消息布局解析）**：只接受 `is_bigendian=false`、x/y/z 字段为 `FLOAT32` 的布局；
  校验 `row_step ≥ width·point_step`、`len(data) ≥ height·row_step`、字段偏移在 `point_step` 内；
  其他格式（大端、FLOAT64 等）**显式拒绝**而不是误读；至少有 `min_valid_points`（默认 10）个
  有限 xyz 点，全 NaN 云按无效输入拒绝；
- **行填充先重排再变换**：`row_step > width·point_step` 的云会先按**每行起始地址**重排为密集布局，
  再交给 TF 变换。原因：本环境安装的 `tf2_sensor_msgs.do_transform_cloud()` 按连续 `point_step`
  遍历 `width*height` 个点，读端若沿用输入布局就会把填充字节当成点、并丢掉真实点。
  适配层不接受这种不确定性：检查、重排、变换、输出都有单测覆盖（含填充区写 90 的样例）。
- TF 必须存在（`require_tf=true`），否则该帧拒绝；
- `map→odom` 平移 > 0.5 m 或旋转 > 0.35 rad 判为**定位跳变**，默认**锁止到整组重启**
  （`jump_latch_duration=0`）。

**地图停更 = 一次任务失效**（两轮 P1 修正）：GridMap 只在"配对通过 + 非空 + 有有效点"时发布
`grid_map/cloud_update` 心跳；`shadow_guard` 要求心跳与其 stamp 都在 `max_map_age`（默认 0.5 s）内。
心跳一旦超时（且此前确实更新过），guard **不只是临时归零**，而是：

1. 记录锁止编号 `latched_task_id = 当前最大 task_id`；
2. 发布 `planning/reset` 撤销任务并**锁止**输出（`map_latched`，按 `revoke_repeat_period` 重复撤权）；
3. 只有出现**编号更大的新任务授权**（`planning/task_active`，`task_id > latched_task_id`）
   **且**地图已恢复时，才解除锁止。

**没有关闭开关**：地图过期**始终**输出零并撤销+锁止。曾经用 `revoke_on_map_stale` 同时控制
"是否撤销"和"是否允许继续输出"，关掉它会连"过期地图仍放行 0.4 m/s"一起放开（复审反例），
因此该参数已被删除。

因此心跳恢复本身不会重新放行旧速度——必须有新任务。启动阶段（从未收到过心跳）只归零、不锁止，
因为此时也不存在任务。**I1 必须核对**：心跳按云的 `header.stamp` 计时，真实云的 stamp
需与本机时钟同尺度；若上游用传感器时钟或别的 epoch，需先对齐（或显式放宽 `max_map_age`），
否则门控会一直判过期。

不健康时：停止发布 `body_pose`/`sensor_pose`/`cloud`，并按 `reset_repeat_period` 发布
`planning/reset` 撤销任务；跟踪器也会因 odom 超时停车并遗忘轨迹。
**启动阶段（从未健康过）不发布 reset**：此时还没有任务可撤销，而且会永久关掉 Mode 2 的自动起步
（FSM 的 `preset_started_` 一旦被取消就置 true）。
**恢复后不自动恢复旧任务**，必须重新下发目标/路线（FSM 会分配新的 `task_id`）。

跨进程任务安全仍是**已有实现边界**：`TaskAuthorization` 与 `Bspline` 没有共同任务字段，
因此不能把现有 cancel 测试当成完整的跨进程安全证明；影子阶段允许"整组重启并清空任务"。

## 六、影子隔离保证

- `shadow_guard` 只创建 `cmd_vel_shadow` 一个速度发布者；**不存在** `/cmd_vel`、`/cmd_vel_remap`
  或 `/sentry_scan/cmd_vel` 发布者（构造上如此，并由判据脚本核对 ROS 图）；
- guard 周期检查上述禁止话题是否被**影子命名空间内**的节点发布；若是则归零并记录 `graph_violation`。
  **外部**发布者（并存 Nav2 的 `controller_server` 等）只统计数量、不阻断影子——在线影子观察的前提
  就是与旧导航共存（P2 修正）；
- `cmd_vel_shadow` 的 `linear.z`、`angular.x/y` 一律置零；`angular.z` 由
  `zero_yaw_candidate`（默认 true）置零，MCU 保留朝向所有权；
- launch 不启动 `open_loop_controller`（它直接发布模拟里程计，不是实车接口）。

## 六点五、现场在线影子（2026-10-09 起的主路径）

没有 ROS bag，改为**实车在线只读影子**（机器人静止即可开始）：

```bash
python3 scripts/onsite_inspect.py --duration 20     # 采集实际话题/类型/QoS/频率/TF/控制发布者
bash scripts/run_shadow_onsite.sh preflight_only    # 预检：输入话题存在、影子未重复启动、谁在控底盘
bash scripts/run_shadow_onsite.sh                   # 启动 /sentry_scan（无底盘输出）
bash scripts/onsite_check_safety.sh idle|motion     # 影子未接真实控制入口
python3 scripts/onsite_send_goal.py --frame odom --x 2.0 --y 0.0
```

现场流程与必答问题见[在线影子运行手册](../migration/onsite_shadow_runbook.md)。
**待现场确认**（本页相应条目仍标 UNKNOWN）：实际话题/命名空间、类型与 QoS、
`world`↔规划系关系、odom 参考点与朝向含义、速度所在坐标系、真实云 stamp 与时钟尺度、
雷达/机体/规划系 TF 链、底盘控制话题的发布者名单。在线影子阶段**不向实车注入故障**。

## 七、回放（B3）

没有录包时用[合成输入](../../scripts/fake_rm_inputs.py)验证契约；有录包时：

1. 设置**独立的** `ROS_DOMAIN_ID`（不是车辆的 domain），并让 `replay:=true use_sim_time:=true`；
   必须有 `/clock`（`ros2 bag play --clock`）。`replay:=true` 会拒绝未设置/为 0 的 domain。
2. 把录包中的控制话题重映射到隔离名称（禁止发到车辆），例如
   `--remap /cmd_vel:=/replay/unused_cmd_vel /cmd_vel_remap:=/replay/unused_cmd_vel_remap`；
   影子入口自身不订阅这些话题。
3. 录包需要包含：`/Odometry_transformed`、`/LIVO2/imu_propagate`、`/cloud_registered`、`/tf`、`/tf_static`。
4. 回放结果必须标注"**真实数据回放，尚未实车验收**"；I2 的真实影子验收在拿到录包前保持未完成。

## 八、测试与证据

```bash
bash scripts/test_shadow_entry.sh adapter_math        # 坐标/杆臂/yaw=90° 数学单测
bash scripts/test_shadow_entry.sh no_inputs           # 启动隔离 + 失效关闭
bash scripts/test_shadow_entry.sh mode1_goal          # Mode 1 → 候选速度
bash scripts/test_shadow_entry.sh pairing_mismatch    # GridMap 严格配对拒绝
ros2 run sentry_scan_adapter check_inputs --duration 10 --planning-frame odom
```

场景、判据与逐条结果见[影子验收证据](../testing/shadow_acceptance.md)。
每个场景的日志与影子 CSV 在 `log/shadow/<场景>.<时间戳>/`（`log/` 不入库，本地保留）。

## 九、仍需用户/电控提供（不阻塞影子开发，阻塞实车门槛）

1. 现车实际启动命令、overlay 顺序；一段含 odom、传播 odom、原始云、TF 的短录包。
2. 确认参考中心/旋转轴/机械最低点与雷达外参是否仍对应当前车（当前 `R=0.26/H=0.25` 为用户给定基线）。
3. 电控 `vel_x/vel_y` 的坐标系、正方向、单位；实际运行的旋转策略。
4. 固件命令断流停车时限、急停与人工接管方式、首轮平地速度/加速度上限。
