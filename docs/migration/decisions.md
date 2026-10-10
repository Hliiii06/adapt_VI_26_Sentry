# 架构决策记录

状态词：CONFIRMED = 源码或用户明确事实（注明来源）；ACCEPTED = 用户认可方向；
PROPOSED = 实施细节建议；SUPERSEDED = 被后续决策或说明覆盖（写明被谁取代）。

| ADR | 日期 | 决定 | 状态与边界 |
|---|---|---|---|
| 001 | 2026-10-01 | 该轮只更新文档与 C 方案规划，不构建运行、不改导航代码 | **SUPERSEDED**：实施与仿真已在其后执行，见 012/014 与[实施报告](implementation_report.md) |
| 002 | 2026-10-01 | 文档维护在本仓库，参考源码独立只读 | CONFIRMED，用户选择；覆盖 `build.md` 原文的文档位置 |
| 003 | 2026-10-01 | RM 参考 main | CONFIRMED；用户拉取后更新至 `7dfe71a`，旧 `82d0741` 基线被 003 取代（变更见[归档](../archive/baseline_update.md)） |
| 004 | 2026-10-01 | ROS2 SCAN `103bce4` 为接口依据；ROS1 仅辅助注释 | CONFIRMED，任务要求 |
| 005 | 2026-10-01 | 保留既有感知定位/底盘基线，Nav2 留作对照回退 | C 计划中的 PROPOSED 边界；不主动修复已工作系统 |
| 006 | 2026-10-01 | 采用 C：目标/参考路线 + SCAN 空间避障 + 哨兵全向执行 | **ACCEPTED，用户明确认可**；具体包/参数/接口仍为 PROPOSED |
| 007 | 2026-10-01 | 先核对输入、执行、停止契约，再接硬件 | PROPOSED 实施顺序；I1/I2 落实中，见[交接 B](real_robot_handoff.md) |
| 008 | 2026-10-01 | 三维导航就是 SCAN 当前实现，不是完整自由三维/地形规划 | CONFIRMED，用户澄清；此前把地形/拓扑全局算法列为必做前置的建议 SUPERSEDED |
| 009 | 2026-10-01 | 当前 RM 配置已实车通过 RViz 2D goal 规划并运行 | CONFIRMED，**用户验证**；未在 SCAN 侧复测，不推广为 SCAN 已可用 |
| 010 | 2026-10-01 | 小陀螺等部分行为由电控实现，主机源码相关分支未必执行 | CONFIRMED，用户说明；协议/坐标/固件超时仍待核实 |
| 011 | 2026-10-01 | `new_msg` 是接口草稿，不作为首期依赖 | CONFIRMED CMake 生成范围；未来接口迁移独立评审 |
| 012 | 2026-10-01 | 先独立 SCAN 全向适配与 PCD/RViz 闭环仿真，审查后再接 RM | ACCEPTED，用户指定顺序，替代旧 P0–P5；S0–S3 已完成 |
| 013 | 2026-10-01 | 保留三种模式，适配 odom 朝向、碰撞检查、全向运动及必要关联代码 | ACCEPTED 实施方向；局部核心修改纳入范围，不是广泛重构许可 |
| 014 | 2026-10-01 | DeepSeek harness 实现代码与仿真，随后 Codex 审查 | CONFIRMED，用户安排；各轮结果见[实施报告](implementation_report.md)与[归档证据](../archive/README.md) |
| 015 | 2026-10-01 | 首版全朝向保守包络，后续可选朝向预测精细模型 | PROPOSED→已实施为**单圆柱**包络；精细模型仍未做，见[SCAN 架构](../architecture/scan_planner.md) |
| 016 | 2026-10-02 | 先打通指定洞口与坡道，增加从入口连续跟随的有界支撑层选择，保留洞顶与侧墙 | **ACCEPTED**，用户明确同意；已实施，见[真实 PCD 修复](../testing/pcd_route_fix.md)。不扩展 Gazebo/全场地形规划，不授权实车输出；缩小模型仅诊断 |
| 017 | 2026-10-07 | 交接 A 整理边界：只整理文档、归档历史、纠正陈旧表述；不搬代码、不改导航行为、不删失败记录 | **ACCEPTED（本次任务）**；映射与验证见[整理报告](cleanup_report.md) |
| 018 | 2026-10-07 | 实车影子接入 B：I1 输入契约、I2 adapter/影子 launch/候选速度/健康门控，不驱动车辆 | **ACCEPTED 并已实现** B1/B2，见[影子输入契约](../interfaces/shadow_input_contract.md)与[验收](../testing/shadow_acceptance.md)；I3 驱车与 I4 决策迁移另行授权 |
| 019 | 2026-10-07 | 影子规划系统一用 `odom`；`body_frame=base_link`，默认不加参考中心偏移 | **PROPOSED**：`base_link` 是否等于几何中心/旋转轴 UNKNOWN，I1 用实车/录包核对后才能冻结 |
| 020 | 2026-10-07 | 速度源 `world` 帧无 TF 时置零并写诊断，不静默当作机体中心速度；杆臂需显式配置 | **ACCEPTED（安全降级）**：`velocity_frame_alias` 只能在校对数值关系后填写；角速度缺失另有开关 |
| 021 | 2026-10-07 | GridMap 严格 sensor/cloud 配对默认关闭，仅影子 launch 打开 | **ACCEPTED（最小改动）**：保持仿真既有行为不变，避免改动影响已有回归 |
| 022 | 2026-10-07 | 影子层禁止透传 yaw 候选与未定义分量，且不下发 `/cmd_vel` | **ACCEPTED**：`yaw_candidate_enabled=false`、`linear.z/angular.x/y` 置零；guard 周期检查 ROS 图 |
| 023 | 2026-10-07 | 适配层"从未健康过"时不发布 `planning/reset`；Mode 2 自动起步等待首帧云 | **ACCEPTED（最小修正）**：启动阶段无任务可撤销，发 reset 会永久关掉 Mode 2 自动起步；空图上规划会连续触发动态可行性失败。仿真定向回归通过 |
| 024 | 2026-10-07 | 目标/参考路线必须经 `task_adapter` 变换到规划系；空/未知 frame 明确拒绝 | **ACCEPTED（审查 P1）**：FSM 不按 header 变换，直接接会把 map 坐标当 odom。Mode 2 参数航点仍约定为规划系（配置期输入） |
| 025 | 2026-10-07 | 输入健康必须覆盖"点云有效性"与"规划地图实际更新" | **ACCEPTED（审查 P1）**：拒绝全 NaN/结构异常的点云；GridMap 发布 `grid_map/cloud_update` 心跳，guard 以 `max_map_age` 门控 |
| 026 | 2026-10-07 | 影子保护层只禁止**影子命名空间内**的节点发布真实控制话题 | **ACCEPTED（审查 P2）**：与 Nav2 并存是在线影子的前提；外部 `/cmd_vel` 发布者只统计、不阻断 |
| 027 | 2026-10-07 | 失效/取消判据必须证明"事件前确有运动 + 限时归零 + 整窗为零" | **ACCEPTED（审查 P2）**：新增 `check_shadow_stop.py`；恢复输入后旧任务不得复活 |
| 028 | 2026-10-07 | 未来时间戳超容差即拒绝且不写入历史 | **ACCEPTED（审查 P2）**：`max_future_stamp` 默认 0.05 s；坏 stamp 不得污染后续时间检查 |
| 029 | 2026-10-07 | 地图心跳停更 = 任务失效：撤销 + 锁止，直到**新任务**且地图恢复 | **ACCEPTED（复审 P1）**：只临时归零会在心跳恢复后重新放行旧速度；guard 记录 `latched_task_id` 并按 `task_id` 更大者解锁 |
| 030 | 2026-10-07 | 停车判据必须覆盖观察窗首尾、有限值，并按**实际事件时刻**计时 | **ACCEPTED（复审 P2）**：注入方发布 `test/fault_marker`；判据检查首帧/末帧覆盖、样本有限值、以标记时刻计算停车延迟 |
| 031 | 2026-10-07 | 点云按 PointCloud2 布局解析；不支持的布局显式拒绝 | **ACCEPTED（复审 P2）**：只接受 `is_bigendian=false` + x/y/z 为 FLOAT32；大端/FLOAT64/截断数据拒绝 |
| 032 | 2026-10-07 | 带行填充的云先重排为密集布局再变换 | **ACCEPTED（三轮复审 P2）**：`tf2_sensor_msgs.do_transform_cloud()` 按连续 `point_step` 遍历，读端沿用输入布局会把填充当点；检查/重排/变换/输出均有单测，并有 `padded_cloud` 全链场景 |
| 033 | 2026-10-07 | 地图过期始终停车并撤销+锁止，删除 `revoke_on_map_stale` | **ACCEPTED（三轮复审 P2）**：该开关同时决定"是否撤销"和"是否允许输出"，关掉会放行过期地图下的 0.4 m/s 候选 |
| 034 | 2026-10-09 | 无 ROS bag：I2 验证改为**实车在线只读影子**；暂停扩建仿真/Gazebo | **ACCEPTED（用户决定）**；离线回放步骤保留备用，真实数据仍需现场实测 |
| 035 | 2026-10-09 | 现场工具一律只读且不改实车节点；故障注入只针对影子链路 | **ACCEPTED（用户要求）**：`onsite_inspect/run_shadow_onsite/onsite_send_goal/onsite_check_safety/onsite_record`；不得停 LIO/雷达/Nav2 |
| 036 | 2026-10-09 | 接管采用"单一速度源 + 默认撤权 + 独立闸门"，默认不改 RM 源码 | **PROPOSED（待审查/批准）**：优先用 UART `twist_topic` 参数与运行期切换；确需改 RM 时单独列最小改动并再次申请 |
| 037 | 2026-10-09 | 安全自检按 (命名空间, 节点名) 判定；外部 `/uart_node` 等只统计不判失败 | **ACCEPTED（四轮复审 P2）**：`get_node_names()` 在本环境返回不带命名空间的短名，必须用 `get_node_names_and_namespaces()`；新增 `external_uart_coexist` 场景 |
| 038 | 2026-10-09 | 在线采集器持续发现话题，`/tf_static`（及 `/map`）用 transient_local 订阅 | **ACCEPTED（四轮复审 P2）**：先启动实车再采集时 volatile 订阅收不到静态外参；新增 `onsite_late_inputs` 场景与 `check_onsite_report.py` |
| 039 | 2026-10-09 | 断流/恢复验证用影子专用可暂停输入闸门，不重启、不停实车节点 | **ACCEPTED（四轮复审 P2）**：`input_gate:=true` + `onsite_pause_inputs.py`；闸门发 `test/fault_marker`，判据按实际事件时刻计延迟；场景 `input_pause_gate` |
| 041 | 2026-10-09 | 里程计消息已在规划系时直接用其位姿，不查同 stamp 的动态 TF | **ACCEPTED（实车发现）**：RM `TfTransformer::odom_callback` 先发 odom、后广播 `odom→base_link`；按消息时刻查只能拿到上一周期样本，`tf_future_tolerance=0.05` 下每帧被拒 → `health_ok` 恒 false。新增 `odom_in_planning_frame`（默认 true）+ 静态 TF 修正参考点；未改 RM |
| 043 | 2026-10-10 | 点云变换由适配层自己做，输出只含 xyz 的密集云；不再用 `do_transform_cloud` | **ACCEPTED（现场发现）**：现场 `/cloud_registered` 为 PCL 风格 48 字节布局（字段间有空洞），`tf2_sensor_msgs.do_transform_cloud()` 报 `PointFields and structured NumPy array dtype do not match` 并整帧被拒。改为按字段偏移解析 + numpy 刚体变换 + 输出 `point_step=12` 的 xyz 云 |
| 044 | 2026-10-10 | `.rviz` 必须随包安装，并在 launch/预检里显式检查 | **ACCEPTED（现场发现）**：`setup.py` 只装 `*.launch.py`/`*.yaml`，launch 却用 share 下的 `.rviz` 打开 RViz → `rviz2 -d <不存在>` 静默显示空界面。新增 `rviz_config` 回归场景与预检 |
| 042 | 2026-10-09 | `tf_future_tolerance` 按实测周期取 0.15 s（原 0.05） | **ACCEPTED（实车发现）**：`/Odometry_transformed` 9.95 Hz → 周期 0.1005 s；`sensor_pose`/云路径对同一动态 TF 的固有偏差可达 0.1 s。查询偏差记录在 `tf.last_lookup_delay_s`，非静默放宽 |
| 040 | 2026-10-09 | 录包必须校验落盘与消息数，异常返回非零 | **ACCEPTED（四轮复审 P2）**：`timeout --signal=INT` 收尾 + 检查 `metadata.yaml` 的 `message_count`；失败不再打印"录制结束" |

方案 C 获认可不等于已经实现；允许后续独立任务范围内的局部 SCAN 修改，不批准顺带修改固件或 RM 定位。
无需再次询问是否选择 C；后续只在范围扩展或必要硬件参数缺失时确认具体事项。

## 2026-10-07 当前任务切换

- **ACCEPTED（用户要求）**：暂停扩建仿真，整理仓库资料，恢复实车替换主线；DeepSeek 实施，Codex 后续审查。
- 交接 A 已完成（ADR 017）；交接 B 的 B1/B2 已实现、B3 合成输入验收完成（ADR 018–023），交付后停止等 Codex 审查。
- 真实 RM 数据/录包影子运行与 I3 驱车仍需用户批准；影子入口没有任何下发底盘命令的开关。
- 参考仓库只读边界不变；实车运行不由规划文档自动授权。
