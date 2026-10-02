# 全向哨兵适配实施报告（S0–S3）

实施范围：把 SCAN-Planner ROS 2（main `103bce4`）纳入本仓库并完成全向哨兵适配与
PCD/RViz 闭环仿真。**未接实车、未修改 RM 的 LIO/配准/TF/Nav2 参数/串口/固件。**
基线输入见 [S0 基线](s0_baseline.md)，测试结果见 [S3 结果](../testing/s3_results.md)。

## 第二轮：Codex 审查后的修正

Codex 审查认为方向正确但**不能认定 S0–S3 已验收**，并提出 4 项缺陷 + 3 项判断。
本轮的修正如下（详细证据见 [S3 结果](../testing/s3_results.md) 第四节）：

| 缺陷 | 修正 |
|---|---|
| **取消后机器人重新运动**（已用日志反证） | FSM 也订阅 `planning/reset`：取消时清 `have_target_`/`trigger_`、清 `local_data_.start_time_`、回 `WAIT_TARGET`，此后不再发布轨迹。跟踪器加**取消闩锁**：记录取消时刻，拒绝 `start_time` 早于该时刻的在途轨迹。验证：取消后 6 s 内 605 个命令样本全为 0 |
| **过期轨迹被当成新轨迹从头执行** | `closed_loop_controller` 改为**拒绝**而非重置：过期（`elapsed > start_time_align_limit`）拒绝、`start_time` 在未来（`> future_time_tolerance`）拒绝；未提供 `start_time` 时接受但告警说明无法判陈旧 |
| **测试脚本可能误杀其它进程** | `scenario.sh` 用 `setsid` 把仿真放进独立进程组，清理只 `kill -- -PGID`；断流测试用 `pkill -g PGID`，不再用宽泛的 `pkill -f 进程名` |
| **测试误判通过** | 判据移到 `scripts/check_stop.py`：从**实际事件时刻**计时，要求时限内归零**且**观察窗内持续为零，事件前必须有非零命令，失败返回非零。记录器不再按场景名分流（改为 `--send-goal`/`--cancel-after` 显式驱动），并真正等待观察窗 |

对 Codex 三项判断的处理：

- **地面处理**：按意见**降级为演示方案**——参数、日志、文档统一标注"绝对高度切图，
  非地面识别，会删除矮结构，不得用于实车"；`map_pub` 额外发布未过滤对照云
  `/sentry_sim/global_cloud_raw`，RViz 新增对照显示；并用受控场景量化了代价。
  正式的局部地面高度/地面分割本轮**未实现**。
- **碰撞验收**：新增 4 张几何明确的合成地图（封闭房间 + 带缺口隔墙），覆盖
  "宽于车体通过 / 窄于车体拒绝 / 中心线不碰但车体边缘会碰时拒绝 / 矮障碍必须仍然拦停"。
  这三类拒绝均由 `A-star failed`（膨胀占据图无通路）驱动，可行性失败为 0，
  因此可以主张碰撞逻辑本身有效。
- **RViz**：仍标为**未验证**，措辞改为"无界面闭环已有运行记录；RViz 交互待验收"。

新增脚本：`scripts/check_stop.py`（停车判据）、`scripts/check_passage.py`（通过/拒绝判据）、
`scripts/make_test_maps.py`（合成地图）、`scripts/summarize_launch_log.sh`（日志摘要）。

## 第三轮：地形分离与高度跟随

用户验收 RViz 后反馈"坡度不明显"。排查确认根因不是 RViz 也不是缺 Gazebo：
**运动模拟器的 z 恒定、且上一版绝对高度删点把地面整层删掉了**（631508 点只留 177762）。
详见 [地形分离与高度跟随](../testing/terrain_following.md)。

改动：

| 文件 | 改动 |
|---|---|
| `scripts/prepare_terrain_map.py`（新） | 本地地面估计：每格低分位数 + 稳健迭代拟合二次曲面（RMS 0.05 m），障碍 = 高于地面 ≥ 0.08 m；输出障碍云 / 地形表面 / 地面网格 |
| `plan_manage/include/plan_manage/ground_height_map.h`（新） | 地面高度网格加载与双线性查询，供模拟器与 FSM 共用 |
| `plan_manage/src/go2_kinematic_sim.cpp` | 新增 `ground_grid_file`/`body_height`；z = 地面(x,y) + body_height（由地形推导，不抄轨迹 z） |
| `plan_manage/src/scan_replan_fsm.cpp`、头文件 | 新增 `grid_map.ground_grid_file`；Mode 1 目标高度改为**目标处地面 + body_height** |
| `simulator/map_generator/src/map_publisher.cpp` | 新增 `ground_file` → 发布 `ground_surface`（**仅显示，绝不喂给 SCAN**）；高度窗口未删点时降为 INFO |
| `launch/sentry_sim.launch.py`、`launch/sentry_sim.rviz` | 新增 `ground_file`/`ground_grid_file` 参数；RViz 增加"Terrain surface (colour = height)"显示 |
| `scripts/make_terrain_maps.py`（新） | 生成 10°/20°/30° 合成坡道 + 解析式地面网格（角度给定，可定量） |
| `scripts/make_terrain_route.py`（新） | 从地面网格生成 Mode 2/3 路线，含沿线净空检查 |
| `scripts/check_terrain.py`（新） | 高度跟随判据：上升量、与地形偏差、反向下降，失败返回非零 |

验证：三个合成坡道的上升量与 `tan(角度)×3 m` 完全一致（0.529 / 1.092 / 1.732 m）；
真实场地路线上升 0.140 m。**未做动力学、未做地形规划**，详见该文的局限一节。

## 改动清单（相对上游 `103bce4`）


### 修改的文件

| 文件 | 改动 | 原因 |
|---|---|---|
| `plan_env/include/plan_env/grid_map.h` | `getInflateOccupancy(pos,yaw)` 在 `offset≈0` 时只查询单个竖直圆柱；新增 `safety_margin_` | 全向底盘的运动方向不等于机头朝向，原来的双圆柱近似依赖轨迹切线 yaw |
| `plan_env/src/grid_map.cpp` | 新增 `grid_map.safety_margin` 参数；膨胀半径 = 半径 + 余量；打印生效包络 | 余量参数化；把生效包络写进日志便于核对 |
| `plan_manage/src/closed_loop_controller.cpp` | 取消「先对齐 yaw 才平移」的门槛；新增 `yaw_mode`(hold/align/spin)、`spin_rate`、`odom_timeout`、`start_time_align_limit`、`future_time_tolerance`；新增 `planning/reset` **取消闩锁**；**过期/未来时间的轨迹明确拒绝**；轨迹合法性与 odom 有效性校验 | 全向适配的核心；上游在 `|yaw_error|>0.8` 时冻结轨迹并只转向，RM 的 UART 又不转发 `angular.z`，会导致永久冻结。取消闩锁与陈旧拒绝见第二轮修正 |
| `plan_manage/src/go2_kinematic_sim.cpp` | `kMaxVYawLimit` 1.0→1.5；新增可选一阶加速度限制 `max_acc_xy`/`max_acc_yaw`；发布实际角速度 | 允许 MPPI 的 `wz_max=1.5`；让速度不能瞬间跳变，闭环跟踪误差才有意义 |
| `plan_manage/src/scan_replan_fsm.cpp` | 订阅 `planning/reset`，新增 `resetTask()`（取消目标、清轨迹时间基准、回 `WAIT_TARGET`）；包络 marker 带上 `safety_margin`；`offset≈0` 时不重复发布重合圆柱，改为补机头朝向箭头 | 只清跟踪器无法停止任务——规划器会继续发轨迹，机器人会重新运动；RViz 显示与碰撞检查同一语义 |
| `plan_manage/include/plan_manage/scan_replan_fsm.h` | 新增 `self_safety_margin_`、`reset_requested_`、`reset_sub_`、`resetCallback`/`resetTask` | 同上 |
| `simulator/local_sensing/src/pointcloud_render_node.cpp` | 新增 `sensor_offset_x/y/z`、`sensor_roll/pitch/yaw`；射线起点与方向改由 `sensor2world = body2world × sensor2body` 给出；`world→sensor` TF 与 `sensor_pose` 改用同一外参 | 上游只有硬编码 `lidar_pitch` 且忽略安装平移，射线原点被当作机体原点 |
| `simulator/map_generator/src/map_publisher.cpp` | 高度带过滤 `keep_z_min`/`keep_z_max`（**显式标注为演示地图的绝对高度切图**，日志打出删除点数与原始等效高度）；新增 `publish_raw_cloud` 发布未过滤对照云；过滤后为空则报错 | 剔除地面点，否则地板会被判为障碍。按 Codex 意见降级为演示方案并提供对照（见 S3 结果第五节） |
| `plan_manage/CMakeLists.txt`、`package.xml` | 安装新的 RViz 配置；删除本仓库不存在的 `go2_description`/`odom_visualization`（及未用的 `robot_state_publisher`/`xacro`）依赖声明 | 保持 manifest 与实际内容一致 |

### 新增的文件

| 文件 | 用途 |
|---|---|
| `config/sentry_planner.yaml` | SCAN 规划参数：包络 0.26/offset 0、高度带 ±0.125、`body_height` 0.125、`max_vel` 1.0 |
| `config/sentry_controllers.yaml` | 跟踪器与运动模拟器参数：MPPI 限速、`yaw_mode`、odom 超时、仿真加速度 |
| `config/sentry_simulator.yaml` | 地图发布（演示用高度过滤、z 归零）与雷达渲染（真实外参、FOV、量程） |
| `config/sentry_waypoints.yaml` | Mode 2 航点示例 |
| `config/sentry_reference_path.yaml` | Mode 3 参考路线示例 |
| `launch/sentry_sim.launch.py` | 统一入口：`/sentry_sim` 命名空间、三模式、参数校验 |
| `launch/sentry_sim.rviz` | RViz 配置：地图/局部云/占据/膨胀/包络+朝向/轨迹/2D Goal Pose |

未改动的上游文件保持逐行一致；`src/simulator/Utils/`（含 55 MB Go2 网格）未纳入。

## S1：全向适配

### 1. 取消强制朝向对齐

上游 `closed_loop_controller` 的 `cmdCallback()`：若 `|yaw_error| > heading_error_threshold(0.8)`
则 `publishExecutionFrozen(true)` + `publishStop(yaw_command)` 并 **直接 return**——
既不推进 `exec_time`，也不发平移速度。对全向底盘这是致命的：机身可以任意朝向平移，
却被迫先原地转向；而 RM 的 UART 不转发 `angular.z`（见 [底盘接口](../interfaces/chassis_interface.md)），
朝向永远无法到位，于是永久停在冻结态。

适配后：**任何 yaw 误差都不阻塞 XY 运动**，朝向与平移完全解耦。

### 2. 朝向策略参数化（`yaw_mode`）

| 取值 | 行为 | 用途 |
|---|---|---|
| `hold`（默认） | 保持接收轨迹瞬间的机身朝向，`wz = kp_yaw · (hold_yaw − odom_yaw)` | 哨兵横移：车头方向不变 |
| `align` | 跟踪轨迹切线朝向，但**不再阻塞平移** | 需要车头朝行进方向时 |
| `spin` | `wz = spin_rate`（限幅） | 「边转边走」包络测试 |

`hold` 用比例项而不是硬性 `wz=0`，这样在有扰动时仍能回正。

### 3. 速度转换

世界系前馈 + 位置比例反馈后，按 **实际 odom yaw** 旋转到机体系：
`vx_body = c·vx_w + s·vy_w`，`vy_body = −s·vx_w + c·vy_w`。这是全向跟踪能正确横移的关键；
上游此处已经正确，改动只是去掉了它前面的门槛。

### 4. 碰撞包络全向保守化

所有碰撞查询最终都走 `GridMap::getInflateOccupancy(pos, yaw)`（`bspline_optimizer` 的
碰撞代价/可行性检查、FSM 安全检查、`dyn_a_star::checkOccupancy`、RViz marker 都调用它）。
因此把 `double_cylinder_offset` 设为 0 就让**全部入口**同时变为与朝向无关的单圆柱包络：

- 有效半径 = `double_cylinder_radius`(0.26) + `safety_margin`(0.0) = 0.26 m
- 高度带 = 轨迹 z ± `obstacles_inflation_z_*` = 0.125 ± 0.125 → `[0, 0.25]` m

代码里显式判断 `offset < 1e-9` 时只做一次单圆柱查询，避免靠参数巧合表达语义。
启动日志会打印实际生效的包络，便于审查：

```
Collision envelope: single vertical cylinder, radius=0.260 m (robot 0.260 + margin 0.000),
z=[-0.125, +0.125] around the query point; orientation independent.
```

**未采用**「按当前 yaw 预测未来轨迹姿态」的精细模型：当前 yaw 只代表现在，
不能无依据铺到整条未来轨迹；全朝向保守包络是首版更安全的选择（PROPOSED 的精细模型留待后续）。

### 5. FSM 时序

上游用 `planning/go2_execution_frozen` 让 FSM 把 `local_data_.start_time_` 往后推，
实现「冻结期间轨迹时间不走」。适配后跟踪器在 `hold/align/spin` 下都恒发 `false`，
因此冻结不再触发，FSM 时序逻辑保持不变且自洽——这是最小改动，没有删除该机制。

另外**不再无条件把 `exec_time` 清零**：收到新轨迹时用消息自带的 `start_time` 对齐
（`exec_time = clamp(now − start_time, 0, duration)`），与 FSM 的
`local_data_.start_time_` 时序一致；偏移超出 `start_time_align_limit`(0.5 s) 时回退到 0 并告警。

### 6. 保护与停止

| 保护 | 实现 |
|---|---|
| 无效轨迹 | 校验 order/点数/knots 数量关系、有限值、时长；不合法则拒绝并保留旧轨迹 |
| 无效 odom | 非有限位置、四元数模长过小 → 拒绝该帧 |
| odom 陈旧 | 超过 `odom_timeout`(0.5 s) → 停车并**遗忘**轨迹，恢复后不会重新激活旧命令 |
| 取消 | 新增 `planning/reset`(std_msgs/Bool)，置 true 即停车并清除轨迹 |
| 仿真命令断流 | `go2_kinematic_sim` 的 `cmd_timeout`(0.3 s) 内无新命令则速度归零 |
| 重启不恢复旧速度 | 模拟器初速度为 0、跟踪器初始无轨迹 |

## S2：PCD + RViz 闭环仿真

```text
PCD ──map_pub──> /sentry_sim/global_cloud
                      │
         body_pose ───┴──> pcl_render_node ──> /sentry_sim/cloud（world 系）
                                                      │
                                                      v
                                            scan_planner_node（地图 + 规划）
                                                      │ planning/bspline
                                                      v
                                        closed_loop_controller ──> /sentry_sim/cmd_vel
                                                      │                    │
                                                      └── body_pose <── go2_kinematic_sim
```

**闭环是真速度闭环**：跟踪器输出机体系 vx/vy 与 yaw rate，运动模拟器积分速度得到位姿，
再作为 `body_pose` 反馈给 SCAN。**没有**使用 `open_loop_controller`（它直接把样条位置
复制成模拟里程计，不能作为跟踪验收依据）。

### 地图与地面处理

`map_pub` 先整体下移 `map_offset_z = -0.30`，再按 `keep_z_min = 0.0` 丢弃地面点，
`keep_z_max = 1.5` 丢弃无意义的顶部：

```
Height filter [0.000, 1.500] m: 631508 -> 177762 points
Loaded 25216 PCD points from /home/hzq/pcd_map/rmuc2026_field.pcd
```

（第二行是 0.1 m 体素降采样后的发布点数。）

这样 **z=0 就是可行驶地面**，轨迹 z=0.125 是机体中心，碰撞查询覆盖 `[0, 0.25]`。
**这是本实施最重要的一个取舍**：该 PCD 的地面散布到 0.22 m，不过滤则地板本身
就是障碍、任何位置都判碰撞；代价是矮于约 0.30 m 的原始障碍也会被一并剔除。

### 雷达外参与局部观测

`pcl_render_node` 现在按真实外参生成观测，启动即打印：

```
Lidar extrinsic in body frame: xyz=(-0.0300, 0.0770, 0.0601) rpy=(0.00, 0.00, 0.00) deg
```

射线起点从「机体原点」改为「机体系下的雷达位置」，方向为 `body2world × sensor2body`
的旋转。`/sentry_sim/lidar_pose`（发送给 SCAN 的 `sensor_pose`）与渲染使用同一外参，
避免出现「云按一个原点、射线按另一个原点」的隐蔽错位。

### 命名空间与隔离

所有节点运行在 `/sentry_sim` 下，`cmd_vel` 只存在于该命名空间：

```
/sentry_sim/{body_pose, lidar_pose, cloud, cmd_vel, move_base_simple/goal,
             initial_path, planning/bspline, planning/reset,
             planning/go2_execution_frozen, self_inflation, grid_map/*}
```

`/cmd_vel`、`/Odometry_transformed`、UART 相关话题完全没有出现在仿真图里。

### 三种模式全部保留

| 模式 | 入口 | 校验 |
|---|---|---|
| Mode 1 | RViz `2D Goal Pose` → `/sentry_sim/move_base_simple/goal` | 需要先收到 body_pose 才能确定目标高度 |
| Mode 2 | `keypoints_file` → `fsm.waypoints` | 启动时校验 YAML 存在、元素个数是 3 的倍数、**首航点不与初始位姿重合** |
| Mode 3 | `reference_path_file` → `reference_path_publisher` → `/sentry_sim/initial_path` | 启动时校验 YAML 存在；路线地面 z + `body_height` 只加一次 |

Mode 2 的首航点校验是本实施新增的 fail-fast：起点=终点会让多项式全局轨迹退化，
上游表现为「反复重规划 1000 次后急停」，很难定位。现在直接报错并说明原因。

### RViz 入口

`launch/sentry_sim.rviz` 由上游 `default.rviz` 生成：移除 Go2 RobotModel，话题全部
命名空间化，Fixed Frame = `world`，`2D Goal Pose` 工具指向
`/sentry_sim/move_base_simple/goal`，并新增「Sentry envelope + heading」Marker 显示
（`/sentry_sim/self_inflation`，同一话题里包含包络圆柱与机头箭头）。

## 环境适配（非源码改动）

本机 `~/.bashrc` 把海康 MVS SDK 的 `/opt/MVS/lib/64` 放在 `LD_LIBRARY_PATH` 最前面，
其 `libusb-1.0.so.0` 缺少 `libusb_set_option` 符号，会让 PCL 的 `libpcl_io` 在运行期
解析失败，`map_pub` 与 `pcl_render_node` 启动即退出。`scripts/run_sentry_sim.sh`
在启动前剔除 `/opt/MVS/` 路径；本仿真不需要 MVS。

## 与原文档建议的差异

| 原建议 | 实际做法 | 理由 |
|---|---|---|
| 可调整包名 | 保留 `go2_kinematic_sim` 可执行名 | 该节点实际已是通用全向运动积分器；改名会牵动 launch/CMake/参数而不带来功能收益，改在文档中说明语义 |
| 新增 XY follower | 改造既有 `closed_loop_controller` | 复用其速度转换与 FSM 时钟契约，改动面更小 |
| 建议的隔离命名空间 `/sentry_scan/*` | 用 `/sentry_sim/*` | 仿真阶段命名，接车阶段（I1）再定实车命名空间 |
