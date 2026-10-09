# Codex 任务书：读取现场采集目录 `collect_20261009_205335` 并完成影子接入参数确认

对象：实车电脑上的一次只读采集结果。目标是把"只有实车在跑才能确定"的事实读出来，
填好影子入口的参数，并为**静止状态下的 RViz 对齐检查**做准备。
本任务书是自包含的；执行前先读本仓库 [AGENTS.md](../../AGENTS.md) 与
[在线影子运行手册](onsite_shadow_runbook.md)。

---

## 0. 一句话任务

读 `COLLECT=~/下载/collect_20261009_205335`，产出
**①现场实测事实表（写进 `docs/testing/onsite_field_2026-10-09.md`）**、
**②一份可直接复制的影子启动命令（填好真实参数）**、
**③未决问题清单**；然后（且仅在那时）按手册做静止对齐检查。

## 1. 硬边界（越线即停，不要"为了跑通"而放宽）

1. **不发布任何底盘/速度命令**；不接 `/cmd_vel`、不做接管；本轮只读。
2. **不停止、不重启、不修改**实车共用的任何节点（LIO、TF、Nav2、串口、电控、雷达）。
3. 不修改 `../VI_26_Sentry`（RM）与 `../SCAN-Planner*`；实施代码只写在本仓库。
4. 不启动模拟里程计、`open_loop_controller`、Gazebo；不发布替代真实定位的 TF。
5. **不缩小真实碰撞包络**；不得用仿真支撑面/诊断尺寸代替真实尺寸。
6. 首轮只做**平坦、空旷区域**；不测洞口、坡道、小陀螺。
7. 采集目录**不得**提交入库（可能含串口设备名、内网地址）；只在本地分析。
8. 结论必须标注 `CONFIRMED`（现场实测/文件行号）/`INFERRED`（由证据推出）/`UNKNOWN`；
   不得把源码静态配置当成实车运行事实。

## 2. 输入：采集目录

路径：`~/下载/collect_20261009_205335`（`~/下载` 即 `~/Downloads`）。
若目录不完整（本次采集在第 7/9 步被卡住过），先用**已修复版**脚本补齐：

```bash
cd ~/adapt_VI_26_Sentry && git pull
bash scripts/onsite_digest.sh ~/下载/collect_20261009_205335        # 只打印摘要，最快
bash scripts/onsite_collect_all.sh                                  # 需要重采时（约 2 分钟，只读）
# 注意：新版本对每个节点单独 timeout，不会再卡在第 7 步
```

文件清单与含义（`COLLECT` 指上面目录）：

| 文件 | 含义 |
|---|---|
| `basic.txt` | 时间、内核、主机名、`ROS_*`/`RMW_*` 环境变量（判断 domain、RMW 实现） |
| `nodes.txt` / `node_info.txt` | 运行中的节点及其发布/订阅（判断哪些属于原系统） |
| `topics.txt` / `topic_info_all.txt` | 话题与类型、每个话题的发布者/订阅者与 QoS |
| `inspect/report.txt` `report.json` | 20 s 只读采集报告：关键话题的类型/频率/接收年龄/stamp 年龄/frame、TF 全表、控制话题发布者 |
| `tf_static_once.txt` | `/tf_static` 的父子帧（外参，含雷达-机体） |
| `tf_echo_*.txt` | 关键 TF 对（`odom→base_link`、`odom→camera_init`、`camera_init→base_link`、`odom→world`…） |
| `frames_*.pdf` / `.gv` | `view_frames` 生成的 TF 树 |
| `hz_*.txt` | 各关键话题的实测频率 |
| `echo_header_*.txt` | 各话题 `header`（frame_id 与 stamp 来源） |
| `echo_odom_pose.txt` / `echo_velocity_twist.txt` | 定位位姿与速度分量（判断参考点/朝向/速度坐标系） |
| `cloud_stats.txt` | 点云布局（`point_step/row_step/is_bigendian/fields`）、有限点数、x/y/z 范围、距原点 <0.6 m 点数 |
| `params/*.yaml` | 各节点运行参数快照（含 `use_sim_time`、`twist_topic`、话题名、frame） |
| `shadow_health.txt` | **影子 `health_ok=False` 的具体原因**（每通道 `count/rejected/frame/age_s`） |
| `shadow_control_audit.txt` | 控制话题的发布者 vs 订阅者（谁在控制底盘） |
| `bag/`（可选） | 本次采集的短录包 |

## 3. 逐项读出事实（建议顺序，全部只读）

统一约定：`COLLECT=~/下载/collect_20261009_205335`。

### A. 图与话题（判断原系统构成）

```bash
cat "$COLLECT/nodes.txt"
sed -n '1,80p' "$COLLECT/topics.txt"
sed -n '/== 关键话题 ==/,/== TF 动态/p' "$COLLECT/inspect/report.txt"
```

要回答：真实输入话题名是否就是 `/Odometry_transformed`、`/LIVO2/imu_propagate`、`/cloud_registered`？
类型分别是什么？有无命名空间前缀？频率多少？

### B. 时间戳与时钟（最常见的"不健康"根因）

```bash
grep -E "stamp_age|recv_age|frames" "$COLLECT/inspect/report.txt" | head -20
cat "$COLLECT/echo_header_Odometry_transformed.txt" "$COLLECT/echo_header_cloud_registered.txt"
grep -riE "use_sim_time" "$COLLECT/params" | head
```

判据：`stamp_age` 稳定在 ±0.1 s 内 → 与本机时钟同尺度（`CONFIRMED`）；
若持续 >0.5 s 或为负得离谱 → 时钟/epoch 不同（`CONFIRMED` 不符），
**不要**擅自放宽 `max_source_age`/`max_future_stamp`，先定位来源（`use_sim_time`？传感器自带时钟？）。

### C. TF 链与规划系选择（决定 `planning_frame`）

```bash
grep -E "^  " "$COLLECT/tf_static_once.txt" | head -40
sed -n '/== TF 动态 ==/,/== 控制话题发布者/p' "$COLLECT/inspect/report.txt"
grep -m1 -A3 "Translation" "$COLLECT/tf_echo_odom_base_link.txt"
grep -m1 -A3 "Translation" "$COLLECT/tf_echo_odom_camera_init.txt"
```

**规划系选择规则（按顺序，取第一个成立的）**：

1. 若存在 `odom → base_link` **且** 点云 frame（通常 `camera_init`）能通过 TF 链连到 `odom`
   → `planning_frame=odom`（当前默认，优先）；
2. 否则若存在 `camera_init → base_link`（或 `camera_init → base_footprint`）而 `odom` 缺失
   → `planning_frame=camera_init`；
3. 否则若 `map → odom` 或 `map → base_link` 存在，且点云也能连上 → `planning_frame=map`；
4. 都不成立 → 记为 `UNKNOWN`，**先不要跑规划**，把 TF 树（`frames_*.pdf`）发回。

同时记录 `task_frame`（任务消息的坐标系数）：默认 `map`。若 TF 里没有 `map`，
则任务必须直接发在 `planning_frame` 下（用 `onsite_send_goal.py --frame <planning_frame>`）。

### D. odom 的参考点与朝向含义

```bash
cat "$COLLECT/echo_odom_pose.txt"
grep -m1 -A6 "Translation" "$COLLECT/tf_echo_odom_base_link.txt"
grep -m1 -A6 "Translation" "$COLLECT/tf_echo_base_footprint_lidar_link.txt"
```

要回答：`/Odometry_transformed` 的位姿参考点是**雷达**还是**机体中心**（看它与
`base_link`/`lidar_link` 的静态外参是否为零）；朝向是否是机体 yaw；`base_link` 与几何中心是否重合。
→ 这决定适配器是否需要 `body_center_offset_xyz`（`rm_input_adapter` 参数）。

### E. 速度坐标系（`world` 问题）

```bash
cat "$COLLECT/echo_velocity_twist.txt"
grep -iE "world" "$COLLECT/inspect/report.txt" "$COLLECT/tf_echo_odom_world.txt" "$COLLECT/tf_echo_world_odom.txt" 2>/dev/null | head
```

判据：`/LIVO2/imu_propagate` 的 `header.frame_id` 是 `world`。

- 若 TF 里**存在** `world`（例如 `world→odom`）→ 记 `CONFIRMED`，把该关系记录到契约；
- 若**不存在** → 保持适配器默认行为（速度降级为零，日志 `velocity_frame_unresolved(world)`）。
  这**不会**让 `health_ok=false`，但会让候选速度恒为零；**不得**仅凭 header 名字设置
  `velocity_frame_alias`，除非有数值证据证明 `world` 与某 frame 等价（例如同一时刻的
  `tf2_echo` 数值 + 用户口径确认）。

### F. 传感器配对（决定 `strict_sensor_pairing` 容差）

从 `inspect/report.txt` 的关键话题表取 `/cloud_registered`（或 `/pointcloud`）与
`sensor_pose`（适配器输出的 `/sentry_scan/sensor_pose`）的 stamp；两者的差值就是
"云与射线原点的配对误差"。若差值 >0.02 s 且稳定，记录实测值，作为配对容差的依据
（默认 `strict_sensor_pairing=false`，即不因配对失败拒绝；只在证据充分时才收紧）。

### G. 点云布局与尺度（决定 `min_valid_points` 与包络）

```bash
cat "$COLLECT/cloud_stats.txt"
```

要回答：`point_step/row_step`（是否有行填充——适配器会自动重排）、`is_bigendian`、
字段类型是否 `FLOAT32`、`is_dense`、有限点数（取其**最小值的 1/10 量级**作为
`min_valid_points` 的参考，当前默认 10）、z 的范围（判断雷达安装高度与地面高度）、
距原点 <0.6 m 的点数（>0 说明把车体自身扫进来了，需要用户确认是否已在 LIO 侧滤除）。

### H. 控制话题与单一速度源

```bash
cat "$COLLECT/shadow_control_audit.txt"
grep -A6 "== 控制话题发布者" "$COLLECT/inspect/report.txt"
grep -E "cmd_vel|twist_topic" "$COLLECT/params"/*.yaml | head -20
```

已由用户确认（**现场实测**）：`/cmd_vel` 的发布者是导航栈 `controller_server`，
该话题订阅者都用 `geometry_msgs/msg/Twist`。仍需在数据里确认：

- `behavior_server`（恢复行为）是否也发布 `/cmd_vel`；
- `controller_server` 空闲时其发布者是否仍存在（用户那次看到发布者数为 0，
  要区分"Nav2 未启动"与"空闲时不注册发布者"）；
- `uart_node` 消费的话题名（从 `params` 里找 `twist_topic`）。

### I. 影子 `health_ok=False` 的原因（**本次最关键的阻塞项**）

```bash
cat "$COLLECT/shadow_health.txt"
grep -E "rejected|unhealthy|degraded|no TF|pairing" "$COLLECT/inspect_stdout.txt" 2>/dev/null | head
```

按 `shadow_health.txt` 的 `原因:` 行逐条对照下表，给出结论与修复参数：

| 原因片段 | 含义 | 处理 |
|---|---|---|
| `X: no valid sample within 0.50s (count=N rejected=0)` | 该话题没进来 | 话题名/命名空间错 → 用 `ODOM_TOPIC`/`VELOCITY_TOPIC`/`CLOUD_TOPIC` 覆盖 |
| `rejected>0` 且含 `source stamp is ... old` | 时间戳尺度不符 | 按 §B 结论处理，**不擅自放宽** |
| 含 `no TF` | 规划系选错或 TF 缺失 | 按 §C 重新选 `planning_frame` |
| 含 `pairing` | 云与射线原点时间差超限 | 按 §F 记录实测差值 |
| `velocity_frame_unresolved(world)` | 不是健康错误 | 见 §E，候选速度会恒为零 |

## 4. 必须产出（写成文件，不要只在对话里说）

### 4.1 `docs/testing/onsite_field_2026-10-09.md`

用下面模板，每条给出**证据来源（文件名 + 行号或字段）**与 `CONFIRMED/INFERRED/UNKNOWN`：

```markdown
# 现场实测事实表（2026-10-09，实车静止，只读采集）

采集目录：collect_20261009_205335（本机分析，不入库）
采集时间：<从 basic.txt 抄>   实车系统状态：<原系统运行中 / 影子入口运行中>

| # | 事实 | 结论 | 证据 | 标记 |
|---|---|---|---|---|
| 1 | 点云话题/类型/频率/frame | | inspect/report.txt:NN | |
| 2 | 定位话题/类型/频率/frame | | | |
| 3 | 速度话题与 `header.frame_id` | | | |
| 4 | 时钟尺度（stamp_age） | | | |
| 5 | TF 链（列全 parent→child） | | tf_static_once.txt / report.txt | |
| 6 | **规划系选择** | | | |
| 7 | odom 参考点与朝向含义 | | | |
| 8 | 速度坐标系与 `world` 关系 | | | |
| 9 | 云与 sensor_pose 配对误差 | | | |
| 10 | 点云布局/有限点数/是否含自身 | | | |
| 11 | 控制话题发布者（单一速度源） | | shadow_control_audit.txt | |
| 12 | `use_sim_time` 与串口话题实测值 | | params/*.yaml | |
| 13 | `health_ok=False` 的根因与修复参数 | | shadow_health.txt | |

## 影子启动参数（据上表填写）
PLANNING_FRAME=... ODOM_TOPIC=... VELOCITY_TOPIC=... CLOUD_TOPIC=...
robot_height=... robot_radius=... min_valid_points=... safety_margin=...

## 未决（需要用户/助手回答）
- 碰撞包络真实尺寸（长×宽、最高点、最低点、雷达相对机体中心 x/y/z 与朝向）
- 急停触发方式与 MCU 速度超时归零时间
- `world` 与 odom/camera_init 的数值关系
- IMU 速度是机体中心还是雷达安装点（杆臂）
- 平坦空旷场地与安全员
- 是否允许把 `hnurm_uart` 的 `twist_topic` 指向闸门输出（接管阶段意向）
```

### 4.2 一份可直接复制的影子启动命令

形如（参数取自上表，未确定的用默认并在注释里标 `UNKNOWN`）：

```bash
ODOM_TOPIC=/Odometry_transformed \
VELOCITY_TOPIC=/LIVO2/imu_propagate \
CLOUD_TOPIC=/cloud_registered \
PLANNING_FRAME=<据 §C> \
bash scripts/run_shadow_onsite.sh start_rviz:=true
```

### 4.3 更新文档（不要新建重复汇报）

- [影子输入契约](../interfaces/shadow_input_contract.md)：把仍标 `UNKNOWN` 的条目按实测更新；
- [影子验收证据](../testing/shadow_acceptance.md) 的实车段落：记录本次实测与未决项；
- [进展](../migration/progress.md)：一行说明本轮读到了什么、还剩什么。

## 5. 之后才允许的下一步（仍是只读）

只有 4.1/4.2 完成后，才按手册 §2–§4 做**静止**对齐检查：

```bash
python3 scripts/onsite_inspect.py --duration 20
bash scripts/onsite_diagnose.sh                 # health 必须变成 True，否则回到 §3.I
# RViz（Fixed Frame = 规划系）里确认：真实云 /sentry_scan/cloud / 包络 / 机体 / 目标 是否重合
python3 scripts/onsite_send_goal.py --frame <规划系> --x 2.0 --y 0.0
bash scripts/onsite_check_safety.sh motion      # 影子速度出现；控制话题上仍无影子发布者
```

任何一步出现异常（方向反、路径穿墙穿地、影子出现在控制话题）→ 立即停止并回传日志。

## 6. 提交与隐私

- 提交前先跑：`git grep -nIE "(BEGIN [A-Z ]*PRIVATE KEY|password[[:space:]]*[:=]|/dev/tty(USB|ACM|THS)[0-9]*)" $(git rev-list --all)`
  ——**公开仓库**，历史也会公开；
- **只提交** `docs/`（事实表与参数）与必要的脚本改动；
  `~/下载/collect_*`、`log/` 下的任何东西都**不要**入库；
- 提交信息写清"现场实测（用户提供数据）"与未决项，不要把 `UNKNOWN` 写成已确认。

## 7. 完成后回传什么

1. `docs/testing/onsite_field_2026-10-09.md` 全文；
2. 4.2 的启动命令；
3. `shadow_health.txt` 里 `原因:` 那一行的原文（判断是否已能转成健康）；
4. 仍未决的问题清单（尤其包络尺寸、急停、`world` 关系）。

**未授权事项**：发布底盘命令、接管速度、驱动车辆、修改 RM。这些须经用户明确同意与 Codex 审查。
