# S0 实施基线

记录日期：2026-10-01。本文件是 S1–S3 的输入快照：把用户给出的机器人/地图/限速参数、
源码来源和改动边界固定下来，后续结论都以本基线为准。

## 源码来源与边界

| 项 | 值 |
|---|---|
| 上游 SCAN 仓库 | `../../SCAN-Planner-Ros2`（= `/home/hzq/SCAN-Planner-Ros2`），main `103bce48bd9de783511d20e286c5e6299b79e47a` |
| 纳入本仓库的包 | `src/planner/{bspline_opt,path_searching,plan_env,plan_manage,scan_planner_msgs,traj_utils}`、`src/simulator/{local_sensing,map_generator,mockamap}` |
| 未纳入 | `simulator/Utils/*`（go2_description 约 55 MB 网格、odom_visualization、pose_utils、waypoint_generator）、`go2_gait_publisher` 的启动 |
| 上游仓库状态 | 只读复制，未修改、未提交；`git status` 保持干净 |
| RM 参考 | `../VI_26_Sentry` main `7dfe71a5dbac8bd9b14cb618f0df960942611156`，只读取参数，未修改 |
| ROS 1 参考 | `../../SCAN-Planner`，仅用于算法理解 |

`go2_description` 等未纳入的包在 `scan_planner/package.xml` 中的 `exec_depend` 已同步删除，
避免 manifest 声明本仓库不存在、也不启动的依赖。

## 用户给定的实施输入

| 输入 | 值 | 来源 |
|---|---|---|
| PCD 地图 | `~/pcd_map/rmuc2026_field.pcd` | 用户指定 |
| 机器人半径 | 0.26 m | 用户指定 |
| 机器人高度 | 0.25 m | 用户指定 |
| 雷达安装外参 | `x=-0.03, y=0.077, z=0.0601, roll=pitch=yaw=0` | 用户指定「按 VI_26_Sentry 真实外参」 |
| 速度上限 | `vx ∈ [-1.0, 1.0]`、`|vy| ≤ 1.0`、`|wz| ≤ 1.5` | 用户指定「按 nav2_params.yaml 的 MPPI 配置」 |

雷达外参的 RM 依据：`VI_26_Sentry/src/hnurm_bringup/params/extrinsic.yaml` 的
`lidar_to_base`（注释写明「描述的是在 base 系下的 mid360 位置」）。

速度上限的 RM 依据：`VI_26_Sentry/src/hnurm_navigation/params/nav2_params.yaml`，
`controller_server.FollowPath`（`nav2_mppi_controller::MPPIController`，`motion_model: "Omni"`）：
`vx_min: -1.0`、`vx_max: 1.0`、`vy_max: 1.0`、`wz_max: 1.50`；`controller_frequency: 100.0`。

## PCD 实测分析

对 `rmuc2026_field.pcd`（binary、float32 xyz、631 508 点）直接解析：

| 项 | 值 |
|---|---|
| x 范围 | -14.860 … 14.860 m |
| y 范围 | -8.020 … 8.020 m |
| z 范围 | -0.060 … 0.900 m |
| z 中位数（z<0.5） | 0.10 m |
| 每个 0.5 m 网格的最小 z | p05 = -0.060，p50 = 0.020，p95 = 0.140 |

**关键发现：地面不是一层薄平面。** z 直方图在 -0.06 … +0.22 m 之间连续分布，
没有单一尖峰，说明地面存在累计噪声/坡度。

这对碰撞检查是决定性的：机动高度带覆盖地面时，占据栅格会把地板本身标成障碍，
并使膨胀层向上延伸到机体高度带内，导致**任何位置都判定为碰撞**。
因此必须先过滤地面点，见 [实施报告](implementation_report.md) 的「地图与地面处理」。

## 场地与示例目标

过滤地面后，在 0.5 m 净空要求下约有 40% 的场地可用。选取 x=-6.0 的一条南北向通道
（约 7 m 长、0.75 m 净空）作为演示走廊，因为沿 +Y 移动而机头保持 yaw=0 正好就是
「车头不变横移」的测试用例：

- 初始位姿：`(-6.0, 1.25, 0.125)`，yaw=0
- Mode 1 / Mode 2 / Mode 3 目标或路线终点：`(-6.0, 7.5)`
- 贴障通过测试的目标：`(-4.25, 2.25)`（独立测得净空 0.344 m）
- 阻挡测试的目标：`(-6.98, 0.58)`（落在障碍点云上）

## 坐标系与高度约定

- 仿真统一使用 `world` 系；`PCD / 传感器原点 / odom / 轨迹 / RViz Fixed Frame` 一致。
- `map_pub` 把 PCD 整体下移 0.30 m（`map_offset_z: -0.30`），使 **z=0 就是可行驶地面**，
  并丢弃 z<0 的地面点。
- 轨迹参考面取机体中心 **z = 0.125**；碰撞查询上下各扩 0.125 m，覆盖完整的
  `[0, 0.25]` 机体高度带。`body_height` 因此是 0.125，地面路线 z 给 0，只相加一次。
- 时钟：全部节点 `use_sim_time=false`（无 `/clock` 来源）；launch 在
  `use_sim_time=true` 时直接报错。

## 尚未由用户确认的量（保持保守/显式标注）

| 量 | 当前取值 | 说明 |
|---|---|---|
| 安全余量 `safety_margin` | 0.0 | 用户未给出，不臆造实车阈值 |
| 仿真加速度上限 | `max_acc_xy=1.0`、`max_acc_yaw=2.0` | 仿真示例值；RM 的 MPPI 加速度上限为 UNKNOWN |
| 跟踪误差/停车时限阈值 | 未设定 | 需用户与硬件条件确认后写入契约 |
| 雷达 FOV / 量程 | `vertical_fov=60°`、`sensing_horizon=30 m`、`polar_resolution=0.3°` | 仿真取值，不是 MID360 数据手册标定 |
