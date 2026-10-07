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

## 二、场景矩阵（合成输入）

| 场景 | 目的（对应交接 B3 测试） | 判据 | 结果 |
|---|---|---|---|
| `adapter_math` | frame/杆臂/速度样例：yaw=90° 旋转、杆臂补偿、缺 TF 降级、时间门控 | 9 个单测全过 | **PASS** |
| `replay_guard` | 回放安全：非隔离 domain 或缺少 `/clock` 时必须拒绝启动 | launch 拒绝并给出原因 | **PASS** |
| `no_inputs` | 启动隔离 + 失效关闭 | 4 个必需节点在跑；无 `/cmd_vel`、`/cmd_vel_remap`、`/sentry_scan/cmd_vel` 发布者；无 `map_pub`/`pcl_render_node`/`go2_kinematic_sim`/`open_loop_controller`/UART 节点；`health_ok=false`；`cmd_vel_shadow` 全零 | **PASS** |
| `healthy_static` | 时间/QoS：真实端点可连接、云与射线原点配对 | `body_pose` 50 Hz、`cloud`/`sensor_pose` 10 Hz、帧均为 `odom`、配对偏差 ≤ 0.02 s；无任务时影子输出为零 | **PASS** |
| `mode1_goal` | 三模式：Mode 1 RViz 目标 | 健康 + 候选速度流过门控（max abs(vxy)=1.00 m/s），`wz=0`、`vz=0` | **PASS** |
| `mode2_waypoints` | 三模式：Mode 2 参数航点 | 健康 + 候选速度（max abs(vxy)=1.00 m/s） | **PASS** |
| `mode3_path` | 三模式：Mode 3 参考路线 | 健康 + 候选速度（max abs(vxy)=1.00 m/s） | **PASS** |
| `cloud_stop` | 输入失效：云中断 | 进入不健康、影子输出为零、发布 `planning/reset` | **PASS** |
| `odom_stop` | 输入失效：odom/TF 停发 | 同上 | **PASS** |
| `tf_stop` | 输入失效：仅动态 TF 停发 | 同上 | **PASS** |
| `stale_stamp` | 时间：旧 stamp 连续重发 | 来源年龄门控生效，进入不健康、输出为零 | **PASS** |
| `stamp_backwards` | 时间：时间戳倒退 | 倒退帧被拒并保持不健康 | **PASS** |
| `localization_jump` | 重定位：map→odom 跳变 1.0 m | 判为跳变、锁止到整组重启、输出为零 | **PASS** |
| `pairing_mismatch` | 时间/配对：sensor_pose 与 cloud 差 0.30 s | GridMap 打印 `strict pairing ... cloud rejected`（本轮 11 次） | **PASS** |
| `cancel` | 任务时序：取消后不重新运动 | 取消后 6 s 观察窗内影子输出全零，输入保持健康 | **PASS** |

### 关键实测数值（本轮日志）

- `healthy_static`：`body_pose` 301 条 / 50.0 Hz、`sensor_pose` 与 `cloud` 各 60 条 / 10.0 Hz，
  接收年龄 ≈ 0.02 s，frame 均为 `odom`，配对检查 PASS。
- `mode1_goal`：`Received trajectory` 出现、`Task authorization GRANTED (task_id=1)`；
  影子 201 个采样，max abs(vxy)=1.0000、max abs(wz)=0.0000、max abs(vz)=0.0000。
- `pairing_mismatch`：`[GridMap] strict pairing: cloud stamp ... vs sensor_pose stamp ... (delta 0.3000s > 0.0200s); cloud rejected`。
- `adapter_math`：`Ran 9 tests ... OK`（含 yaw=90°、杆臂 `v_center = v_point + ω × R·offset`、
  未补偿时降级、`velocity_frame_unresolved` 不计入必需 TF 失效、旧 stamp/倒退拒绝）。

## 三、本轮新增的 SCAN 改动与回归

| 改动 | 默认行为 | 回归 |
|---|---|---|
| `grid_map.strict_sensor_pairing` + `sensor_pairing_tolerance` | **默认 false**，仿真行为不变 | 仿真 `mode1_lateral` / `mode2_waypoints` / `cancel_race` 通过 |
| `closed_loop_controller.yaw_candidate_enabled` | **默认 true**，仿真行为不变；影子置 false 只把 `angular.z` 置零 | 同上（日志确认 `yaw_candidate_enabled=true`） |
| `PlanningVisualization.visualization_frame_id` | **默认空** = 上游 world/map 硬编码，仿真显示不变 | 编译与上述场景通过；RViz 图形交互本环境无法验证 |
| FSM Mode 2 自动起步等待首帧云（`GridMap::hasCloudData()`） | 只影响 `navi_mode=2` 的起步时机 | 仿真 `mode2_waypoints` 通过 |

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
| frame/杆臂/速度样例 | `adapter_math`：yaw=90° 旋转、杆臂 `v_center = v_point + ω × R·offset`、缺 TF 降级 |
| 时间/QoS | `healthy_static`（端点可连接、频率、配对）+ `stale_stamp` + `stamp_backwards` |
| 三模式 | `mode1_goal` / `mode2_waypoints` / `mode3_path` |
| 输入失效 | `cloud_stop` / `odom_stop` / `tf_stop` / `localization_jump` |
| 任务时序 | `cancel`；**延迟旧授权/旧轨迹**由既有仿真 `cancel_race` 覆盖（同一 `TaskAuthorization` + `planning/reset` 实现），影子矩阵未重复 |
| 重定位 | `localization_jump` |
| 碰撞/高度 | 影子 launch 使用真实尺寸参数（R=0.26/H=0.25、单圆柱包络）；**碰撞逻辑本身不在影子矩阵重跑**，仍以受控仿真场景（`gap_edge`/`low_obstacle`/`terrain_tunnel_low`）为证据。真实洞净高与实车最低包络 UNKNOWN |
| 回放安全 | `replay_guard`（非隔离 domain / 无 `/clock` 必须拒绝）+ 契约第七节步骤 |

## 五、未验证 / 未完成（不得写成已通过）

1. **真实 RM 数据影子验收（I2 核心）**：无录包、无场地许可，未做；契约测试只覆盖合成消息。
   `world` 与 `odom` 的数值关系、`/LIVO2/imu_propagate` 的角速度与 IMU 杆臂均 **UNKNOWN**，
   因此速度前馈当前按"降级为零"处理（见契约第四节）。
2. **`body_frame=base_link` 是否等于几何中心/旋转轴**：UNKNOWN，需要用户用实车尺寸/照片核对。
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
