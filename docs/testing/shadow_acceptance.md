# 影子接入验收证据（I1/I2，B3）

日期：2026-10-07。实现见 `src/sentry_scan_adapter`，契约见[影子输入契约](../interfaces/shadow_input_contract.md)。
**结论边界**：以下都是**合成输入 + 本机 ROS 2 Humble**下的契约验收；**没有实车、没有真实 RM 录包**，
因此 **I2 真实影子验收仍未完成**，不得据此宣称实车可用。

## 一、环境与可复现命令

```bash
scripts/build.sh                                   # 10 个包（9 个 SCAN + sentry_scan_adapter）
bash scripts/test_shadow_entry.sh adapter_math     # 坐标/杆臂/yaw 数学单测，不需要 ROS 图
bash scripts/test_shadow_entry.sh mode1_goal       # 其余场景会自行启动影子 launch
```

- 影子场景都在 `ROS_DOMAIN_ID=73`（可用 `SHADOW_DOMAIN_ID` 覆盖），与仿真/实车 domain 隔离；
- 每个场景的 `launch.log`、`fake_inputs.log` 与影子 CSV 在 `log/shadow/<场景>.<时间戳>/`、`log/shadow/shadow_*.csv`（`log/` 不入库，本地保留）；
- 判据脚本：`scripts/check_shadow_graph.py`（ROS 图与门控）、`scripts/check_inputs`（输入健康与配对）、
  `scripts/test_shadow_adapter_math.py`（数学）；任一失败返回非零；
- 合成输入 `scripts/fake_rm_inputs.py` **不发布任何速度命令**，只发 RM 侧已有话题/TF 与测试用任务话题。

若需重跑完整矩阵：

```bash
for s in adapter_math replay_guard no_inputs healthy_static mode1_goal mode2_waypoints \
         mode3_path cloud_stop odom_stop tf_stop stale_stamp stamp_backwards \
         localization_jump pairing_mismatch cancel; do
  bash scripts/test_shadow_entry.sh "$s" || echo "FAILED: $s"
done
```

最近一次完整记录：`log/shadow/final_matrix.log` 的 14 个场景**全部通过**（2026-10-07 17:02–17:08），`replay_guard` 单独执行通过，合计 **15/15**。

## 二、场景矩阵（合成输入，22 个场景）

| 场景 | 目的（对应交接 B3 测试） | 判据 | 结果 |
|---|---|---|---|
| `adapter_math` | frame/杆臂/速度样例 + 未来/倒退时间戳 + 点云有效点 | 11 个单测全过 | **PASS** |
| `replay_guard` | 回放安全：非隔离 domain 或缺少 `/clock` 必须拒绝 | launch 拒绝并给出原因 | **PASS** |
| `no_inputs` | 启动隔离 + 失效关闭 | 必需节点（含 `task_adapter`）在跑；影子命名空间内无禁止话题发布者；无仿真/UART 节点；`health_ok=false`；输出全零 | **PASS** |
| `healthy_static` | 时间/QoS：端点可连接、云与射线原点配对 | 帧均为 `odom`、配对 ≤0.02 s、无任务时输出为零 | **PASS** |
| `mode1_goal` | 三模式：Mode 1 目标（经 `task_adapter`） | 健康 + 候选速度 max abs(vxy)=1.00 m/s，`wz=vz=0` | **PASS** |
| `mode2_waypoints` | 三模式：Mode 2 参数航点 | 健康 + 候选速度 | **PASS** |
| `mode3_path` | 三模式：Mode 3 参考路线 | 健康 + 候选速度 | **PASS** |
| `task_frame_transform` | **坐标转换（审查 P1）**：非单位 `map→odom`、目标与路线在 `map` 下 | 输出 frame=`odom` 且数值等于 `T_odom<-map·p`；未知/空 frame 触发明确拒绝日志且无输出 | **PASS** |
| `nav2_coexist` | **与旧导航并存（审查 P2）**：外部 `controller_server` 发布 `/cmd_vel` | 外部发布者只统计；影子仍放行候选速度（max abs(vxy)=1.00） | **PASS** |
| `cloud_stop` | 输入失效：云中断 | 事件前有运动 → 限时归零 → 整窗为零 | **PASS** |
| `invalid_cloud` | **无效点云（审查 P1）**：持续注入全 NaN 云 | 适配层按有效点拒绝（日志 `finite xyz`）→ 停车 | **PASS** |
| `map_gate` | **地图未更新门控（审查 P1）**：心跳来源切断，适配器仍健康 | 事件前有运动 → 心跳停后归零（`--expect-healthy-after` 断言此时 `health_ok=true`） | **PASS** |
| `odom_stop` | 输入失效：odom/TF 停发 | 运动 → 限时归零 → 整窗为零 | **PASS** |
| `tf_stop` | 输入失效：仅动态 TF 停发 | 同上 | **PASS** |
| `stale_stamp` | 时间：旧 stamp 连续重发（先正常运行再冻结） | 同上 | **PASS** |
| `stamp_backwards` | 时间：时间戳倒退 | 同上 | **PASS** |
| `localization_jump` | 重定位：map→odom 跳变 1.0 m | 同上 | **PASS** |
| `recovery_no_resume` | 任务时序：失效→停车→**输入恢复** | 恢复后 `health_ok=true` 但输出仍全程为零（旧任务不复活） | **PASS** |
| `map_relatch` | **地图锁止（复审 P1）**：心跳 11 s 停、20 s 恢复，**不发新任务** | 停更时撤销+锁止（日志 `LATCHING`）；恢复后未出现 `map latch cleared`，输出到窗末持续为零 | **PASS** |
| `map_relatch_newtask` | **地图锁止解除（复审 P1）**：同上，24 s 发新目标 | 零窗 `[fault+3, 23]` 全零；新任务授权（`task_id` 更大）且地图恢复后运动在 26 s 后恢复 | **PASS** |
| `pairing_mismatch` | 时间/配对：sensor_pose 与 cloud 差 0.30 s | GridMap 打印 `strict pairing ... cloud rejected` | **PASS** |
| `cancel` | 任务时序：取消 | 事件前有运动、限时归零、整窗为零、健康保持为真 | **PASS** |

停车类场景统一由 `scripts/check_shadow_stop.py` 判定：**事件前必须有 ≥0.2 m/s 的候选速度**
（否则场景无效）、最后一个非零样本必须出现在 `事件时刻 + 3 s` 内、`观察窗`内每个样本都为零、
采样间隔 ≤0.5 s；可选 `--expect-healthy-after` 断言门控在健康为真时也生效。

若需重跑完整矩阵：

```bash
for s in adapter_math replay_guard no_inputs healthy_static mode1_goal mode2_waypoints \
         mode3_path task_frame_transform nav2_coexist cloud_stop invalid_cloud map_gate \
         map_relatch map_relatch_newtask odom_stop tf_stop stale_stamp stamp_backwards \
         localization_jump recovery_no_resume pairing_mismatch cancel; do
  bash scripts/test_shadow_entry.sh "$s" || echo "FAILED: $s"
done
```

最近一次完整记录：**22/22 全部通过**（`log/shadow/final3_matrix.log`，2026-10-07 19:13–19:25；`log/` 不入库）。

## 二点五、审查修正（对照 54a1b1d–c1a2440）

| 审查项 | 修正 | 证据 |
|---|---|---|
| **P1 缺少目标/路线坐标转换** | 新增 `task_adapter`：按消息 stamp 把 `task/goal_in`、`task/path_in` 从任意 frame 转到规划系后发布 `goal`、`initial_path`；空/未知 frame 明确拒绝。FSM 的 `move_base_simple/goal` remap 到 `goal`，Mode 3 发布器 remap 到 `task/path_in` | `task_frame_transform`：非单位 `map→odom`（dx=1, dy=1, yaw=0.3）下 goal `(2,1)`→`(0.955,-0.296)`、路线两点转换正确、未知/空 frame 无输出并留拒绝日志 |
| **P1 健康不代表地图有效更新** | 适配层按**消息布局**校验点云（大端/FLOAT64/截断显式拒绝、行填充按行首址、`min_valid_points`）；GridMap 只在"配对通过+非空+有有效点"时发布 `grid_map/cloud_update` 心跳；`shadow_guard` 以 `max_map_age` 门控 | `invalid_cloud`（全 NaN 云被拒、日志 `finite xyz`、停车）与 `map_gate`（心跳切断后撤销+锁止）；单测含全 NaN/行填充/大端/FLOAT64/截断 |
| **P2 保护层阻断与 Nav2 并行观察** | guard 只把**影子命名空间内**节点发布的 `/cmd_vel`、`/cmd_vel_remap` 判为违规；外部发布者只统计并记录 | `nav2_coexist`：外部 `controller_server` 发布 `/cmd_vel` 时，影子仍放行候选速度 |
| **P2 失效停车可能"本来没动"** | 新增 `check_shadow_stop.py`：事件前必须有运动、限时归零、整窗为零、采样**首尾覆盖**且有限值；失效场景一律先发目标 | `cloud_stop`/`invalid_cloud`/`map_gate`/`map_relatch`/`odom_stop`/`tf_stop`/`stale_stamp`/`stamp_backwards`/`localization_jump`/`cancel` 全部先产生 1.00 m/s 候选速度再判停车 |
| **P2 未来时间戳被接受并污染历史** | `max_future_stamp`（默认 0.05 s）：超限拒绝且**不更新** `last_stamp`；追加单测 | `adapter_math` 新增用例：+3600 s 被拒、历史不变、随后正常 stamp 仍被接受 |

修正后既有结论不变：影子入口仍**没有**下发 `/cmd_vel` 的开关，所有场景仍断言
影子命名空间内不存在 `/cmd_vel`/`/cmd_vel_remap` 发布者。

### 实测停车延迟（按注入方标记的实际事件时刻）

| 场景 | 实际事件（marker） | 最后非零 | 延迟 |
|---|---|---|---|
| `cloud_stop` | 12.04 s | 12.40 s | 0.360 s |
| `invalid_cloud` | 12.38 s（nan_cloud） | 12.73 s | 0.351 s |
| `map_gate` | 11.03 s（heartbeat_cut） | 11.39 s | 0.363 s |
| `map_relatch` | 11.38 s（heartbeat_cut） | 11.78 s | 0.396 s |
| `map_relatch_newtask` | 11.05 s（heartbeat_cut） | 11.41 s | 0.363 s |
| `odom_stop` | 12.01 s | 12.46 s | 0.453 s |
| `tf_stop` | 12.04 s | 12.19 s | 0.150 s |
| `stale_stamp` | 11.03 s（stamp_freeze） | 11.51 s | 0.476 s |
| `stamp_backwards` | **6.02 s**（不是固定的 11 s） | 6.39 s | 0.367 s |
| `localization_jump` | 12.03 s | 12.88 s | 0.853 s |
| `recovery_no_resume` | 12.38 s（cloud_stop） | 12.75 s | 0.368 s |

## 二点六、第二轮复审修正（对照 34519cc）

| 复审项 | 修正 | 证据 |
|---|---|---|
| **P1 地图恢复后自动放行旧任务** | 地图心跳停更不再只临时归零：`shadow_guard` 记录 `latched_task_id`、发布 `planning/reset` 并锁止（`map_latched`），只有**编号更大的新任务授权**且地图已恢复才解除 | `map_gate`（停更即撤销+锁止）、`map_relatch`（心跳 20 s 恢复但无新任务 → 到窗末持续为零、无 `map latch cleared`）、`map_relatch_newtask`（24 s 新目标 → 零窗到 23 s、26 s 后恢复运动，实测停更延迟 0.399 s） |
| **P2 停车判据可在观察窗缺失时假通过** | 判据增加：首帧须在启动后 1 s 内、末帧须覆盖到 `--zero-until`、所有样本有限、按真实事件时刻计时 | 注入方在真正注入时发布 `/sentry_scan/test/fault_marker`；判据优先用该标记的本机接收时刻（输出里打印 `fault=... (marker(...))`），`--fault-at` 仅作回退；`map_relatch` 实测 `fault=11.04s (marker)` |
| **P2 点云未按 PointCloud2 布局解析** | 只接受 `is_bigendian=false` + x/y/z 为 `FLOAT32`；校验 `row_step`/`data` 长度与字段偏移；行填充按每行起始地址寻址；其他布局**显式拒绝** | `adapter_math` 新增用例：行填充全 NaN → 0 有效点（此前误报 1）、大端被拒并给出 `is_bigendian` 原因、FLOAT64 被拒、截断被拒、行填充有效云计数正确 |

## 三、本轮新增的 SCAN 改动与回归

| 改动 | 默认行为 | 回归 |
|---|---|---|
| `grid_map.strict_sensor_pairing` + `sensor_pairing_tolerance` | **默认 false**，仿真行为不变 | 仿真 `mode1_lateral` / `mode2_waypoints` / `cancel_race` 通过 |
| `closed_loop_controller.yaw_candidate_enabled` | **默认 true**，仿真行为不变；影子置 false 只把 `angular.z` 置零 | 同上（日志确认 `yaw_candidate_enabled=true`） |
| `PlanningVisualization.visualization_frame_id` | **默认空** = 上游 world/map 硬编码，仿真显示不变 | 编译与上述场景通过；RViz 图形交互本环境无法验证 |
| FSM Mode 2 自动起步等待首帧云（`GridMap::hasCloudData()`） | 只影响 `navi_mode=2` 的起步时机 | 仿真 `mode2_waypoints` 通过 |
| GridMap 发布 `grid_map/cloud_update` 心跳（接受云时） | 新增只读话题；订阅/参数默认值不变；空云不再算作一次更新 | 仿真回归通过（见下） |
| `task_adapter` 新节点 | 影子入口专用；仿真链不含它 | 影子 `task_frame_transform` 通过 |

仿真的定向回归命令与结果：

```bash
bash scripts/scenario.sh mode1_lateral   # 通过
bash scripts/scenario.sh mode2_waypoints # 通过
bash scripts/scenario.sh cancel_race     # 通过
```

## 四、对应交接 B3 测试表的覆盖对照

| 交接 B3 要求 | 覆盖情况 |
|---|---|
| 启动隔离 | `no_inputs`：无模拟 odom/雷达/Go2/UART 节点，且禁止话题无发布者 |
| frame/杆臂/速度样例 | `adapter_math`：yaw=90° 旋转、杆臂 `v_center = v_point + ω × R·offset`、缺 TF 降级、未来/倒退时间戳、点云有效点 |
| 任务坐标转换 | `task_frame_transform`：非单位 `map→odom` 的目标与路线转换 + 未知/空 frame 拒绝 |
| 时间/QoS | `healthy_static`（端点可连接、频率、配对）+ `stale_stamp` + `stamp_backwards` |
| 三模式 | `mode1_goal` / `mode2_waypoints` / `mode3_path` |
| 输入失效 | `cloud_stop` / `invalid_cloud` / `map_gate` / `odom_stop` / `tf_stop` / `localization_jump`（全部经 `check_shadow_stop.py` 证明"先动后停"） |
| 与旧导航并存 | `nav2_coexist`：外部 `/cmd_vel` 发布者不阻断影子 |
| 任务时序 | `cancel`（先动后停）+ `recovery_no_resume`（恢复输入后旧任务不复活）；**延迟旧授权/旧轨迹**由既有仿真 `cancel_race` 覆盖（同一实现），影子矩阵未重复 |
| 重定位 | `localization_jump` |
| 碰撞/高度 | 影子 launch 使用真实尺寸参数（R=0.26/H=0.25、单圆柱包络）；**碰撞逻辑本身不在影子矩阵重跑**，仍以受控仿真场景（`gap_edge`/`low_obstacle`/`terrain_tunnel_low`）为证据。真实洞净高与实车最低包络 UNKNOWN |
| 回放安全 | `replay_guard`（非隔离 domain / 无 `/clock` 必须拒绝）+ 契约第七节步骤 |

## 五、未验证 / 未完成（不得写成已通过）

1. **真实 RM 数据影子验收（I2 核心）**：无录包、无场地许可，未做；契约测试只覆盖合成消息。
   `world` 与 `odom` 的数值关系、`/LIVO2/imu_propagate` 的角速度与 IMU 杆臂均 **UNKNOWN**，
   因此速度前馈当前按"降级为零"处理（见契约第四节）。
2. **`body_frame=base_link` 是否等于几何中心/旋转轴**：UNKNOWN，需要用户用实车尺寸/照片核对。
   另外地图心跳按云的 stamp 计时，I1 必须确认真实云的 stamp 与本机时钟同尺度。
3. **Mode 2 多航点顺序完成**：合成输入里"机器人"不动（fake 不推进位姿），因此只验证了
   起步、规划与候选速度流通，**没有**验证逐点到达与切换。真实录包回放才能覆盖。
4. **RViz 图形交互**：本环境创建不了 OpenGL 上下文，未验证影子 RViz 配置的实际显示。
5. **I3 驱车**：未授权；影子入口没有任何下发 `/cmd_vel` 的开关。

## 六、本轮发现的既有问题（不在本次修复范围）

- **Mode 2 短航点段的可行性失败**：在默认参数（`manager.max_vel=1.0`、`manager.max_acc=0.5`、
  `manager.feasibility_tolerance=0.5`）下，≤1.5 m 的航点段会连续报
  `Dynamic feasibility failed: acceleration ... > 1.500`（999 次）并最终无轨迹；
  同一参数下 ≥2.0 m 的段正常（单段 2.0 m：0 次失败）。
  这是**既有规划器时间分配**的表现，不是影子适配引入；本轮只把测试航点改为 ≥2 m 并记录，
  是否调整时间分配留待 I3 前的专项评审（涉及动力学阈值，不能为让测试通过而放宽）。
- **Mode 2 起步早于地图数据**：修复前 Mode 2 在第一帧 odom 就规划，此时占据图还是空的
  （实测比 adapter 的健康判定还早 0.1 s）。已加"等待首帧云"的门（见第三节），
  仿真回归通过；该门不改变其他模式。
