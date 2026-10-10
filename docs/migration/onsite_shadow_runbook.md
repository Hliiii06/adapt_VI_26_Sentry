# 实车在线只读影子运行手册（On-site Runbook）

日期：2026-10-09。契约见[影子输入契约](../interfaces/shadow_input_contract.md)，
判据与矩阵见[影子验收证据](../testing/shadow_acceptance.md)，接管方案见[实车接管方案](real_robot_takeover_plan.md)。

**授权边界（本轮）**：授权读取实车输入、运行只读影子；**不授权发布真实底盘命令**。
受控驱车必须经 Codex 审查 + 用户明确同意后再安排。
实施代码只在本仓库；`../VI_26_Sentry` 与 SCAN 参考仓库保持只读；
**不修改** LIO、TF、Nav2、串口、电控协议；不启动模拟里程计或 `open_loop_controller`；
不发布替代真实定位的 TF；候选速度只出现在 `/sentry_scan/cmd_vel_shadow`。

证据分级（本页与相关文档统一使用）：

| 标记 | 含义 |
|---|---|
| **本机实测** | 本轮/近期在本仓库用**合成输入**跑出的结果（见影子验收证据） |
| **历史证据** | 归档的静态源码核对与早期仿真结果（见 `docs/archive/`） |
| **待现场确认** | 只有实车在跑时才能确定的事实；本页给出采集命令与回传清单 |

---

## 0.0 现场一次性采集（推荐先跑这个）

```bash
cd ~/adapt_VI_26_Sentry && git pull
bash scripts/onsite_collect_all.sh              # 全只读；约 1~2 分钟
# 产物：log/onsite/collect_<时间戳>.tar.gz —— 直接回传这个包
```

采集器对**每个节点单独限时**（`ros2 param dump` 遇到不响应参数服务的节点会永久挂起，
曾导致卡在第 7/9 步、后面的打包没执行）。若某个节点被 `skip`，不影响其余内容。

已经跑过一次但没打包时，用摘要脚本补：

```bash
bash scripts/onsite_digest.sh ~/下载/collect_<时间戳> --tar
# 终端打印可粘贴摘要；--tar 同时生成 ~/下载/collect_<时间戳>.tar.gz
```

包内含：`nodes/topics/topic_info_all/node_info`、20 s 采集报告（`inspect/report.{txt,json}`）、
TF（`frames_*.pdf` + `tf_echo_*.txt` + `tf_static_once.txt`）、各关键话题 `hz_*.txt` 与
`echo_header_*.txt`、`cloud_stats.txt`（点云布局/尺度/是否含自身）、`params/*.yaml`
（含 `use_sim_time`、串口与控制话题的实际运行值）、以及可选的 `bag/`。
**不要**把包提交到公开仓库（`log/` 已被 `.gitignore` 排除）。

## 0.-1 让 Codex 接手分析采集结果

采集目录在实车电脑上；把 [Codex 任务书](codex_onsite_analysis_brief.md) 交给 Codex 执行，
它会产出 `docs/testing/onsite_field_2026-10-09.md`（现场实测事实表）、填好参数的启动命令与未决问题清单。

## 0. 一次性准备（机器人静止即可）

```bash
cd <本仓库>

# 每个新终端都要先 source；Python 工具（采集/发目标/暂停）依赖这些环境变量
source /opt/ros/humble/setup.bash
scripts/build.sh                              # 10 个包（SCAN 9 + sentry_scan_adapter）
source install/setup.bash                     # 让本仓库包与消息类型可见

# 保持原有实车系统正常运行，然后采集"实际在跑什么"
python3 scripts/onsite_inspect.py --duration 20
# 产物：log/onsite/<时间戳>/report.txt 与 report.json

# 报告自检（可选，但建议把结果一起回传）
python3 scripts/check_onsite_report.py --report log/onsite/<时间戳>/report.json \
  --require-topic /cloud_registered --require-topic /Odometry_transformed \
  --require-tf-static --require-tf "base_link -> base_footprint"
```

**需要回传的内容**（报告里已包含，另可手工复核）：

1. `report.txt` 全文 + `report.json`；
2. `ros2 topic list -t`（报告含关键话题，建议整表）；
3. 关键话题的 `ros2 topic info -v`（发布者/订阅者/QoS）：`/Odometry_transformed`、
   `/LIVO2/imu_propagate`、`/cloud_registered`、`/pointcloud`、`/tf`、`/tf_static`；
4. TF 链：`timeout 5 ros2 run tf2_tools view_frames`（生成 `frames.pdf`）+ 报告中 `/tf`、`/tf_static` 列表；
5. **底盘控制入口**：`ros2 topic info -v /cmd_vel`、`/cmd_vel_remap`、`/cmd_vel_nav`
   （谁在发布、类型是 `Twist` 还是 `TwistStamped`）；
6. 静止时 `timeout 5 ros2 topic hz /cloud_registered` 与 `/Odometry_transformed` 的频率。

**据实测结果要确认的 5 件事**（对应任务二）：

| 要确认 | 看什么 | 判据 |
|---|---|---|
| 点云/定位/速度话题、类型、频率、时间戳、QoS | 报告"关键话题"表 | 频率稳定；`stamp_age` 与本机时钟同尺度（<0.5 s）；QoS 与 SensorDataQoS 兼容 |
| 点云/雷达/机体/规划系的 TF | 报告 TF 两节 + `tf2_echo` | 存在 `odom→base_link`、`base_link→base_footprint`、`base_footprint→lidar_link`（或等效链）；云 frame 能连到规划系 |
| odom 参考点、朝向含义、速度所在坐标系 | `tf2_echo odom base_link`、`/LIVO2/imu_propagate` 的 `twist` | 参考点是雷达/机体？朝向是否等于机体 yaw？速度是 world 系还是 body 系？**不能只按源码假设** |
| 真实底盘控制话题与当前发布者 | `/cmd_vel*` 的发布者名单 | 记录"谁在控制底盘"，影子接入时该名单不得多出 `/sentry_scan/*` |
| ~~碰撞包络实测尺寸~~ | — | **已确认（2026-10-09）**：半径 0.26 m、高 0.15 m 的圆柱；`run_shadow_onsite.sh` 默认已按此设置 |
| ~~急停方式~~ | — | **已确认**：存在可立即停车的急停；**MCU 速度超时归零时限仍 UNKNOWN** |
| `world` frame 与规划系的关系 | TF 列表 + `tf2_echo odom world`（若存在） | 若 `world` 不在 TF 里，保持适配器默认降级（速度置零），**不要**凭 header 名声明等价 |

把确认结果填到本文 §8 的表格，并同步到[影子输入契约](../interfaces/shadow_input_contract.md)
的"待 I1 核对"项。

---

## 1. 保持原有实车系统正常启动

按现场原有流程启动（例如 `relocal_nav.sh` 或你们惯用的终端组合），确认：

```bash
ros2 node list | head -30            # laserMapping / relocation_node / uart / nav2 等应在
ros2 topic hz /cloud_registered      # 真实点云在流
ros2 topic info -v /cmd_vel          # 原有速度源（Nav2 controller/behavior、决策）在
```

本仓库**不参与**这一步：不要用本仓库脚本启动定位、TF、Nav2、串口。

## 2. 单独启动 SCAN 影子入口

```bash
bash scripts/run_shadow_onsite.sh preflight_only   # 只做检查：话题存在？影子没在跑？谁在控底盘？
bash scripts/run_shadow_onsite.sh start_rviz:=true # 正式启动（前台；Ctrl-C 退出）
```

现场入口自带实车包络默认值（半径 **0.26 m**、高 **0.15 m**，用户 2026-10-09 确认，`safety_margin=0`）。
**不要**直接 `ros2 launch sentry_scan_adapter sentry_scan_shadow.launch.py`：launch 自身默认是通用值
`robot_height=0.25`（仿真/回归用）。

> `start_rviz` 默认是 **false**：要按第 3 节在 RViz 里对齐，必须显式加 `start_rviz:=true`。

- 默认输入：`/Odometry_transformed`、`/LIVO2/imu_propagate`、`/cloud_registered`，规划系 `odom`
  （2026-10-09 现场已确认：SCAN 在 `odom` 规划；TF 不使用 `world`，速度保持降级为零）；
- 关键参数：`odom_in_planning_frame:=true`（默认，实车 `TfTransformer` 先发 odom 后发 TF）、
  `tf_future_tolerance:=0.15`（按实测 9.95 Hz 动态 TF 周期定）；
- 若 §0 实测发现话题/命名空间不同，用环境变量覆盖后重启：
  `ODOM_TOPIC=... VELOCITY_TOPIC=... CLOUD_TOPIC=... PLANNING_FRAME=... bash scripts/run_shadow_onsite.sh`
- 无界面：`bash scripts/run_shadow_onsite.sh start_rviz:=false`；
- 期望看到的节点：`/sentry_scan/{rm_input_adapter,task_adapter,shadow_guard,scan_planner_node,closed_loop_controller}`
  （RViz 时再加 `/sentry_scan/rviz2`）；**不会**出现 `map_pub`、`pcl_render_node`、`go2_kinematic_sim`、
  `open_loop_controller`、`uart_node`。

## 3. RViz 同时查看真实点云、机体、SCAN 路径与目标

影子入口自带的 RViz 配置已包含（Fixed Frame = 规划系，默认 `odom`）：

| 显示项 | 话题 | 用途 |
|---|---|---|
| **RM cloud (/cloud_registered, real)** | `/cloud_registered` | 真实点云（现场对齐用） |
| RM filtered cloud (/pointcloud) | `/pointcloud` | 默认关闭，需要时勾选 |
| **RM odom (/Odometry_transformed)** | `/Odometry_transformed` | 真实机体位姿（箭头） |
| Sensor Cloud | `/sentry_scan/cloud` | 适配后进入 SCAN 的云（应与真实云重合） |
| PCD map / Occupancy / Inflated | `/sentry_scan/grid_map/*` | SCAN 实际看到的占据与膨胀 |
| Sentry envelope + heading | `/sentry_scan/self_inflation` | 碰撞包络与机头方向 |
| global/optimal/init/a_star list、Goal | `/sentry_scan/*_list`、`goal_point` | 规划轨迹与目标 |

对齐检查：真实云、`/sentry_scan/cloud`、包络和机体箭头应在同一位置重合；
Fixed Frame 若改成 `map` 仍能正确显示，说明 TF 链完整。

## 4. 只给 SCAN 发目标（不碰 Nav2）

```bash
# Mode 1（在规划系；实际 frame 由 §0 实测决定）
python3 scripts/onsite_send_goal.py --frame odom --x 2.0 --y 0.0

# 若目标来自 map 系，显式声明，由 task_adapter 转换
python3 scripts/onsite_send_goal.py --frame map --x 3.0 --y 1.0

# Mode 3 参考路线（地面 z；body_height 由 SCAN 加一次）
python3 scripts/onsite_send_goal.py --path "0,0,0.0;2,0,0.0;2,2,0.0" --frame odom
```

- 脚本只允许发布到 `/sentry_scan/` 下的话题，写死拒绝其它话题（防止误发 Nav2 入口）；
- RViz 里的 **2D Goal Pose 工具已指向 `/sentry_scan/task/goal_in`**，可代替命令行；
- **不要**使用 Nav2 面板的 "Nav2 Goal"、也不要手动发 `/goal_pose`、`/move_base_simple/goal`——
  那会驱动原有导航（本轮不做）。

## 4.5 影子不健康时怎么查（现场一条命令）

```bash
git pull                      # 现场机器先更新到最新脚本
bash scripts/onsite_diagnose.sh
```

输出依次给出：

1. `/sentry_scan/health` 的**原因字符串**与每通道 `count / rejected / frame / age_s`
   （`rejected` 增长而 `count` 不增长 = 该通道被拒；`age_s=n/a` = 从未收到；`age_s` 偏大 = 中断或时间戳尺度不符）；
2. 适配器最近的拒绝原因（`log/shadow/*/launch.log`）；
3. 门控最近的原因（`log/shadow/shadow_*.csv` 尾部）；
4. **控制话题的发布者/订阅者**（用 `onsite_control_audit.py` 精确区分——不要用
   `ros2 topic info -v | grep`，它会把订阅者 `uart_node` 误显示成发布者）；
5. 最近一次采集报告的头部。

若第 2 步显示 `velocity_frame_unresolved(world)`：这是**设计内的降级**（`/LIVO2/imu_propagate`
的 `header.frame_id` 是 `world`，若 `world` 与规划系之间没有 TF，就没有可信的速度方向）。
它不会让 `health_ok` 变 false，但会让候选速度恒为零；确认 `world`↔规划系关系后才考虑
`velocity_frame_alias`。

## 4.6 2026-10-09 已定位的现场故障（health_ok=False，已修）

现象：影子节点都在、`/cmd_vel` 上没有影子发布者，但 `health_ok=False`、影子输出恒零。

根因（源码 + 现场数据 + 本地复现）：RM `TfTransformer::odom_callback` **先发**
`/Odometry_transformed`、**之后**广播同 stamp 的 `odom→base_link`；适配器按消息时刻查这个动态 TF，
只能拿到上一周期样本（≈0.1 s），超过当时的容差 0.05 s → odom 每帧被拒。

修复后现场应看到：

```bash
bash scripts/onsite_diagnose.sh
# 期望：health_ok=True；odom.pose_source = message(child=base_footprint+static)；
#       tf.last_lookup_delay_s 很小；没有 "needs future data"
```

若仍为 false，把 `原因:` 行、`odom.rejected`、`tf.last_lookup_delay_s`、`cloud.rejected` 一起回传。

## 5. 检查 SCAN 没有发布真实控制命令

```bash
bash scripts/onsite_check_safety.sh idle      # 未发目标：影子输出必须全零
bash scripts/onsite_check_safety.sh motion    # 已发目标：候选速度应流过影子话题
```

人工复核（任何时候）：

```bash
ros2 topic info -v /cmd_vel          # 发布者必须全是原有导航；出现 /sentry_scan/* 立即 Ctrl-C 并回传
ros2 topic info -v /cmd_vel_remap
timeout 3 ros2 topic echo --once /sentry_scan/cmd_vel_shadow   # w z 与 linear.z 恒为 0
```

## 6. 安全退出（不影响原有系统）

1. 在影子终端 **Ctrl-C**（只结束影子入口的进程组）；
2. 确认影子节点消失：`ros2 node list | grep sentry_scan`（应无输出）；
3. 确认原系统仍在：`ros2 node list | head`、`ros2 topic hz /cloud_registered`；
4. 确认底盘控制入口与启动前一致：`ros2 topic info -v /cmd_vel`。

## 7. 可选：录制本次调试

```bash
bash scripts/onsite_record.sh 120          # 默认 120 s；只录制，不发布
# 产物：log/onsite/<时间戳>/bag/ （与同时间段 log/shadow/ 一起回传）
```

录制脚本会核对 `metadata.yaml` 里的消息数：**异常退出、没有落盘或 0 条消息都会返回非零**，
不会再打印"录制结束"骗人。若返回非零，先解决录制问题，不要把空 bag 当证据回传。
需要看特定话题时用 `TOPICS="/Odometry_transformed /cloud_registered" bash scripts/onsite_record.sh 60`。

---

## 8. 影子阶段必须回答的问题（结果填此表）

| # | 问题 | 怎么做 | 判据 | 现场结果 |
|---|---|---|---|---|
| 1 | 点云、机体、目标是否对齐 | RViz 同时开真实云、`/sentry_scan/cloud`、RM odom、Goal | 三者重合；切 Fixed Frame 不跑偏 | 待现场 |
| 2 | 定位朝向与速度方向是否正确 | `tf2_echo odom base_link`；`ros2 topic echo /sentry_scan/body_pose --field twist`；对比实际朝向/运动方向 | 朝向与实际一致；速度符号与实际运动一致（`world` 关系未确认时先记录降级状态） | 待现场 |
| 3 | 地图是否持续更新 | `ros2 topic hz /sentry_scan/grid_map/cloud_update`；RViz 看 Occupancy | 心跳随云频率；无"有云但地图不动" | 待现场 |
| 4 | 路径是否明显穿墙/穿地 | RViz 叠加真实云 + 包络 + optimal_list/global_list | 轨迹不穿云、不穿地；可疑处截图 | 待现场 |
| 5 | 输入失效后影子速度是否归零、任务是否撤销 | **只在影子侧制造失效**（见下），不碰实车节点 | `cmd_vel_shadow` 归零；`planning/reset` 出现 | 待现场 |
| 6 | 输入恢复后旧任务是否不会自动继续 | 同上恢复后不重新发目标 | 影子输出保持零，直到发新目标 | 待现场 |
| 7 | 影子是否始终没接真实控制出口 | 每次操作后跑 `onsite_check_safety.sh` | 控制话题发布者名单里始终没有 `/sentry_scan/*` | 待现场 |

**在线影子不能给实车注入故障**，而且**重启影子会清空旧任务**，不能证明"运行中的断流锁止"。
因此用一个只作用于影子输入的**可暂停闸门**：`input_gate:=true` 时，影子入口在实车话题与适配器之间
插入 `input_pause_gate`（只订阅实车话题、只发 `/sentry_scan/*`），规划器/跟踪器/门控**持续运行**：

```bash
# 终端 A：带闸门启动影子（RViz 可选）
bash scripts/run_shadow_onsite.sh input_gate:=true start_rviz:=true

# 终端 B：发目标，确认候选速度正常
python3 scripts/onsite_send_goal.py --frame odom --x 2.0 --y 0.0
bash scripts/onsite_check_safety.sh motion

# 断流：只暂停转发（实车雷达/定位/Nav2 完全不受影响）
python3 scripts/onsite_pause_inputs.py --pause
# 期望：health 转为不健康 -> 发布 planning/reset -> /sentry_scan/cmd_vel_shadow 归零
ros2 topic echo /sentry_scan/health --once
bash scripts/onsite_check_safety.sh idle

# 恢复：继续转发，但**不发新目标**
python3 scripts/onsite_pause_inputs.py --resume
# 期望：health 恢复为 true，但影子输出仍为零（旧任务已撤销，不会自动续跑）
bash scripts/onsite_check_safety.sh idle

# 新任务才恢复
python3 scripts/onsite_send_goal.py --frame odom --x -2.0 --y 0.0
bash scripts/onsite_check_safety.sh motion
```

闸门在暂停/恢复时会在 `/sentry_scan/test/fault_marker` 发布标记，因此"停车延迟"按**实际事件时刻**计算。
本仓库的合成回归场景 `input_pause_gate` 就是这条链路（实测延迟 0.392 s，恢复后保持零，新任务后恢复运动）。

首轮范围：**只做平坦、空旷区域**；不测洞口、坡道、小陀螺。
规划出路径 ≠ 车辆能安全通过；**不得**缩小真实碰撞包络或使用仿真的支撑面/诊断尺寸。

---

## 9. 与历史证据、待确认项的分工

- **本机实测（本轮）**：`onsite_inspect.py`、`onsite_send_goal.py`、`onsite_check_safety.sh`、
  `onsite_record.sh`、`run_shadow_onsite.sh` 在**合成输入**下的彩排（场景 `onsite_tools`）：
  采集报告可生成、安全自检 idle/motion 通过、目标只进影子入口、能录包。
- **历史证据**：RM/SCAN 源码静态核对（`docs/archive/`、`docs/interfaces/`）与仿真回归；
  它们**不代表**实车运行事实。
- **待现场确认**：§0 的 5 件事 + §8 的 7 个问题；`world`↔规划系关系、IMU 杆臂与角速度、
  真实云 stamp 与时钟尺度、`base_link` 是否几何中心。
