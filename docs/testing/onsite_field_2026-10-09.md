# 2026-10-09 实车输入采集分析

## 结论与范围

**本批完成的是实车输入摸底，不是 SCAN 影子验收，更不是实车驱动验收。**

- **CONFIRMED（用户提供的现场记录）**：定位、速度、点云输入存在，主要 TF 链可查询；原有 Nav2 与 UART 在运行。
- **CONFIRMED（本批观察窗口）**：节点列表没有 `/sentry_scan` 节点，`shadow_health.txt` 等待 4 秒未收到健康消息。不能据此证明影子运行成功，也不能确定未收到消息的唯一原因。
- **CONFIRMED**：`world` 未被本批 TF 查询找到；不能直接把该速度消息当成 `odom` 系速度。
- **CONFIRMED（TF 数值）**：`base_link` 与 `lidar_link` 的静态合成变换为单位变换。**UNKNOWN（物理含义）**：它们是否等于车辆几何中心、实际雷达/IMU 原点以及真实离地高度。
- **UNKNOWN**：真实输入下 SCAN 是否持续更新地图、正确规划、失效撤权并在恢复后保持停车。本批没有相应运行证据。

本轮仅离线阅读采集文件并核对本仓库代码（`40017ec`），未运行 ROS、未构建、未操作实车、未调整参数。现场部署提交未记录，不能假定与分析版本一致。

当前操作入口：[在线影子运行手册](../migration/onsite_shadow_runbook.md)；接口定义：[影子输入契约](../interfaces/shadow_input_contract.md)。本文是这一次现场采集的证据分析，不替代接口契约。

## 1. 原始证据与采集条件

原始目录：`/home/hzq/下载/collect_20261009_205335`。本轮不搬迁、不修改原始资料；以下目录内相对文件名均指该目录。资料未复制入 Git，跨机器阅读需同时保留此采集目录。

| 证据 | 用途 |
|---|---|
| `basic.txt` | 2026-10-09 20:53:35 CST 开始；ROS 2 Humble，`ROS_LOCALHOST_ONLY=0` |
| `inspect/20261009_205456/report.txt`、`report.json` | 20 秒输入观测、端点 QoS、节点、TF、控制发布者 |
| `topic_info_all.txt`、`node_info.txt`、`topics.txt`、`nodes.txt` | 运行图快照与 UART 订阅关系 |
| `tf_static_once.txt`、`tf_echo_*.txt`、`frames_2026-10-09_20.55.07.gv` | TF 数值与拓扑；GV 为 PDF 的可读来源 |
| `hz_*.txt`、`cloud_stats.txt`、`echo_*.txt` | 独立采样窗口中的频率、点云布局与消息样例 |
| `params/TfTransformer.yaml`、`params/PointCloudNode.yaml` | 现场导出的参数，不等同于参考源码默认值 |
| `shadow_health.txt`、`shadow_control_audit.txt` | 影子健康缺失与存在矛盾的控制审计结果 |

命令分时执行，不能将不同文件里的位姿或时间戳当作同一帧配对。环境文件未显示显式 `ROS_DOMAIN_ID`；跨终端 domain/RMW 一致性仍需现场确认。

## 2. 输入话题：哪些已经确认

下表频率和年龄来自同一份 20 秒 inspector 报告；年龄是报告末尾样本，不是全窗口最大延迟。

| 话题 | 类型 / frame | 观测频率 | 末样本 stamp 年龄 | 发布者 QoS |
|---|---|---:|---:|---|
| `/Odometry_transformed` | Odometry / `odom` | 9.95 Hz | 0.049 s | Best Effort / Volatile |
| `/Odometry` | Odometry / `camera_init` | 9.79 Hz | 0.049 s | Reliable / Volatile |
| `/LIVO2/imu_propagate` | Odometry / `world` | 175.40 Hz | 0.004 s | Reliable / Volatile |
| `/cloud_registered` | PointCloud2 / `camera_init` | 8.89 Hz | 0.043 s | Reliable / Volatile |
| `/pointcloud` | PointCloud2 / `base_footprint` | 8.66 Hz | 0.327 s | Best Effort / Volatile |
| `/segmentation/obstacle` | PointCloud2 / `camera_init` | 9.07 Hz | 0.327 s | Best Effort / Volatile |

**INFERRED**：这些发布端的 reliability/durability 与适配器 SensorDataQoS 订阅兼容，且观测到的输入时间戳与采集机时钟接近；这不是端到端影子处理成功证明。

独立 `hz_cloud_registered.txt` 窗口约 7.22 Hz，最大接收间隔 0.400 s；`cloud_stats.txt` 另测约 7.34 Hz。不同窗口频率不同并不自动说明数据错误，但离默认 0.5 s 新鲜度阈值已有一定接近程度。下一次需记录实际影子健康/地图心跳，不应为了消除报警直接放宽超时。

`/map` 在 inspector 中计数为 0，同时注明 `type not auto-subscribed (not in registry)`，并列出了 map_server 的 Reliable/Transient Local 发布端。**不能将该 0 解释为地图没有发布。**

## 3. TF 与机体参考点

采集可见的主要链路：

```text
map → odom → base_link → base_footprint → lidar_link → livox_frame
          └→ camera_init → aft_mapped
                         （world 未在采集的 TF 树中出现）
```

| 变换 | 本批读数 / 解释 |
|---|---|
| `odom → camera_init` | 静态单位变换 |
| `base_link → base_footprint` | 平移约 `(0.030, -0.077, -0.0601)` m，单位旋转 |
| `base_footprint → lidar_link` | 平移约 `(-0.030, 0.077, 0.0601)` m，单位旋转 |
| `base_link → lidar_link` | 由上述两段相加得零平移、单位旋转；这是 TF 定义，不是物理标定结论 |
| `map → odom` | 示例平移约 `(4.096, 1.630, -0.001)` m，yaw 约 `130.698°`；明显不是单位变换 |
| `odom ↔ world` | 两个方向均持续报告 `world` frame 不存在，未得到有效变换 |

多数 `tf_echo` 开头短暂提示 frame 不存在，随后成功输出；这些初始化发现提示不能算持续 TF 缺失。`world` 的查询则一直未成功。静态 TF 时间戳为 0、`hz_tf_static.txt` 没有频率输出，不代表静态 TF 过期；GV 中静态边的 10000 Hz 也不是实测发布频率。

`echo_odom_pose.txt` 的 `/Odometry_transformed` 位置约 `(0.0290, -0.0772, -0.0574)`，与分时采集的 `odom → base_footprint` 接近，而不是 `odom → base_link` 的近零位置。**INFERRED**：该里程计 pose 更接近 footprint 参考点；需要完整 odom（含 child_frame_id）与同时间 TF 再确认。

本仓库 `rm_input_adapter.py:on_odom()` 按时间戳查询 `planning_frame ← body_frame`，`publish_body_pose()` 再应用 `body_center_offset`，并非直接复制输入 odom pose。因此不能仅凭 `/Odometry_transformed` 数值，认定 SCAN 已得到正确的机体中心。

**下一步必须核对**：实际几何中心/旋转轴、雷达和 IMU 位置、底部/顶部相对参考点高度。不要直接把 `body_frame` 改为 footprint 或补一段高度来“对齐”；应先确认测量与既有 TF 语义。地图 z 原点不是天然的地面，当前近零 z 不能解释为车辆中心离地为零。

## 4. 速度与点云的具体边界

### 速度

`echo_velocity_twist.txt` 的单帧线速度约 `(-0.00354, 0.01000, -0.00313)` m/s，角速度为零。该单帧不能证明速度坐标系、旋转时角速度有效性或杆臂补偿正确。

按当前 `rm_input_adapter.py:build_velocity()` 的默认语义，无法解析 `world` 到规划系时会降级为零速度（原因 `velocity_frame_unresolved...`）。这是**代码推断**，本批没有影子健康消息证明实际走到了该分支。零前馈不等于禁止所有候选运动，也不能用健康为真来证明速度语义正确。

不得仅因 `odom → camera_init` 是单位变换就声明 `world=odom`；缺少的是 `world` 的定义与对应证据。

### 点云

`cloud_stats.txt` 记录：

- `/cloud_registered`：小端、FLOAT32 xyz、height=1、point_step=48、无行填充；每帧有限点 11076–11435。示例 width=11349、row_step=544752。布局符合当前适配器支持范围。
- `/pointcloud`：小端、FLOAT32 xyz、height=1、point_step=16、无行填充；有限点 2544–3047。它已经经过障碍分割与高度裁切，不是原始云的无损替代。
- 现场 PointCloudNode 参数：输入 `segmentation/obstacle`，输出 `/pointcloud`，高度范围 `[-0.35, 1.0]`、robot_radius=0.25、sensor_height=0.31。不能未经评估就切换到它来判断洞顶或完整三维障碍。

统计脚本给出的“距原点 <0.6 m 平均约 18 点”**不能单独证明车体自扫描**：尤其 `camera_init` 是地图参考系，不保证原点始终随车；即便在车体系，附近地面/墙也可能进入这个球。需按消息时刻转换到机体系并叠加真实包络检查。也不采纳“有限点最小值的 1/10”作为自动调参依据；本轮不改 `min_valid_points`。

## 5. 控制与影子状态：保留矛盾，不宣布通过

主 inspector 报告列出 `/cmd_vel` 发布端：controller_server 和 behavior_server（后者出现多个端点，不能直接说是多个独立控制进程）。`node_info.txt` 的 `/uart_node` 明确订阅 `/cmd_vel: geometry_msgs/msg/Twist`。

但 `shadow_control_audit.txt` 把所有控制话题写为“当前不存在”并给出 PASS。这与主报告不一致。**UNKNOWN**：是否由发现时序、运行状态改变或采集环境不同造成。本批不能用该 PASS 证明控制链不存在或隔离验收通过。

`shadow_health.txt` 明确失败，主节点列表也没有影子节点。准确结论是：**没有观测到正在运行的影子链路**；“没有影子发布真实命令”在影子未运行时不构成启动隔离测试。

## 6. 下一次现场最小步骤

1. 保持原有系统运行、机器人无导航任务；记录部署提交、ROS domain/RMW，与本次原始目录一并保存。不修改 RM、TF 或底盘参数。
2. 先用尺寸/照片确认第 3 节参考点；无法确认时只做输入观察，不把碰撞包络显示当验收通过。
3. 按[运行手册](../migration/onsite_shadow_runbook.md)在独立终端 source 环境，启动 `run_shadow_onsite.sh input_gate:=true start_rviz:=true`。只输出影子速度，禁止真实底盘接管。
4. **先不发目标**：确认影子节点实际存在、健康原因、真实云与适配云对齐、机体参考点和圆柱包络位置合理、地图心跳持续更新。保留健康全文与 RViz 截图；`world` 问题不要通过虚构 TF 消除。
5. 确认目标工具只发 `/sentry_scan/task/goal_in` 后，在实际可见的空旷位置给目标。记录路径、候选与影子速度；静止车辆不会沿路径运动，这是影子模式的正常边界。
6. 有非零影子速度后再暂停影子输入、恢复输入、不发新目标，验证撤权停车与不自动续跑；不停止原有雷达/LIO/Nav2。暂停时应使用 `check_shadow_graph.py --expect-unhealthy --expect-zero`，不能使用同时要求健康的检查模式。
7. 同一观察窗口重做控制发布者审计，保存完整端点及节点名单。可选录包；至少包含健康、地图心跳、reset/任务授权、候选/影子速度及输入/TF，才能分析失效时序。

本批没有路径、运动或故障事件连续记录，因此不填写停车延迟、路线通过率或影子验收“通过”。I3 底盘输出仍未授权。


---

# 附：本仓库第二轮分析（2026-10-09 晚，含根因与修复）

本节由实施方在本机直接读取原始采集目录后追加，**不修改上文的结论与证据边界**。上文第 5 节的
"没有观测到正在运行的影子链路"仍然成立：那次采集期间影子入口没有运行，因此**没有**健康消息。
但用户更早一次现场自检（19:20 左右）确实观察到了 `health_ok=False` 与影子节点，本节解释该现象并给出修复。

## A. 根因（CONFIRMED：源码 + 现场数据 + 本地复现）

1. **证据（现场）**：`inspect/…/report.txt` 中 `/Odometry_transformed` 为 9.95 Hz、frame `odom`、
   stamp 年龄 0.049 s；`hz_Odometry_transformed.txt` 的样本间隔 0.095–0.105 s（**周期 ≈ 0.1005 s**）。
   动态 TF 只有 `odom→base_link`、`map→odom`、`camera_init→aft_mapped`；
   静态 TF 为 `odom→camera_init`、`base_link→base_footprint`、`base_footprint→lidar_link`、
   `lidar_link→livox_frame`、`base_footprint→back_camera`。
2. **证据（RM 源码，只读）**：`VI_26_Sentry/src/hnurm_bringup/src/tf_transformer_node.cpp` 的
   `odom_callback()` **先** `odom_pub_->publish(...)`，**之后**才
   `tf_broadcaster_->sendTransform(odom→base_link)`，两者使用**同一个 stamp**。
3. **机制**：适配器在收到里程计消息时按**该消息的 stamp** 查询动态 TF `odom←base_link`。
   此刻该 TF 尚未进入本进程的 tf2 buffer，回退到"最新可用 TF"只能是**上一周期**的样本
   （偏差 ≈ 0.1 s）；而当时的容差 `tf_future_tolerance=0.05 s` ⇒ 每帧被判
   `needs future data (delta ≈0.06–0.10s > 0.0500s)` 并拒绝 → odom 通道永远没有有效样本
   → `health_ok=False`、影子输出恒为零。这与用户 19:20 的观察一致。
4. **同一机制的第二处**：`sensor_pose`（`odom←lidar_link`，用于射线原点）在云 stamp 上查询同一动态 TF，
   云 stamp 落在两个 TF 样本之间时同样需要"未来" TF（最长 ≈0.1 s），也会被拒。

## B. 修复（仅本仓库，未改 RM）

| 改动 | 位置 | 说明 |
|---|---|---|
| 里程计已在规划系时直接用其位姿 | `sentry_scan_adapter/rm_input_adapter.py:on_odom()`（新参数 `odom_in_planning_frame`，默认 true） | `/Odometry_transformed` 的 `header.frame_id` 就是规划系 `odom`，无需再查同一时刻的动态 TF；`child_frame_id`（这里是 `base_footprint`）与 `body_frame`（`base_link`）的差异用**静态** TF（`Time(0)`，与发布顺序无关）修正（`_apply_transform_to_pose`） |
| 容差按**实测周期**调整 | `config/shadow_contract.yaml`、launch 默认 `tf_future_tolerance: 0.15` | 0.05 小于 10 Hz 动态 TF 的固有偏差；0.15 = 一个周期 + 余量。查询实际使用的偏差记录在 `tf.last_lookup_delay_s`，不是静默放宽 |

## C. 本地复现与回归（CONFIRMED：本仓库合成输入）

新增两个场景复现实车发布顺序（`fake_rm_inputs.py --tf-after-odom --tf-delay-after-odom 0.03`：
先发 `/Odometry_transformed`，同 stamp 的动态 TF 延后 30 ms）：

| 场景 | 配置 | 结果 |
|---|---|---|
| `odom_before_tf` | 本修复（直通 + 0.15 容差） | `health_ok=True`、候选速度 1.00 m/s、日志中**没有** `needs future data` |
| `odom_before_tf_legacy` | `odom_in_planning_frame:=false` + `tf_future_tolerance:=0.05`（旧行为） | **`health_ok=False`**、影子输出全零、日志出现 `lookup odom<-base_link … needs future data (delta 0.0599s > 0.0500s)` —— 精确复现实车症状 |

## D. 用户确认的现场事实（2026-10-09，记为现场口述证据）

| 事实 | 影响 |
|---|---|
| TF 不使用 `world` 系，按"不存在"处理 | 保持适配器默认：`/LIVO2/imu_propagate` 的 `world` 速度降级为零（**不**设置 `velocity_frame_alias`）。候选速度仍由位置闭环产生 |
| SCAN 在 `odom` 系规划 | `planning_frame=odom`（本仓库默认）**CONFIRMED** |
| IMU 取自 mid360（与雷达同体），雷达基本可当作机体中心 | `imu_to_body_offset_xyz` 视为零、`require_center_velocity=false`；`body_center_offset_xyz` 暂为空 |
| 允许第 7 项（`hnurm_uart` 的 `twist_topic` 指向闸门输出） | 接管阶段可用 launch 参数切换，**不需要改 RM 源码**；本轮不实施 |

## E. 仍然 UNKNOWN（阻塞静止对齐检查的通过判定）

- 车辆**几何中心/旋转轴**与 `base_link` 的真实关系、真实离地高度（TF 定义 `base_link→lidar_link` 为零**不等于**物理标定）；
- 碰撞包络实测尺寸（长×宽、最高点、最低点）→ `robot_height` / `robot_radius` / `safety_margin`；
- 急停触发方式与 MCU 速度超时归零时间；
- 修复后现场 `health_ok` 是否转为 true、地图心跳是否持续（需要带着本修复重跑影子入口）。

## F. 下一步（现场，仍只读）

```bash
cd ~/adapt_VI_26_Sentry && git pull && scripts/build.sh
bash scripts/run_shadow_onsite.sh input_gate:=true start_rviz:=true
bash scripts/onsite_diagnose.sh          # 期望 health_ok=True，且 odom.pose_source=message(child=base_footprint+static)
python3 scripts/onsite_inspect.py --duration 20
```

若 `health_ok` 仍为 false，把 `onsite_diagnose.sh` 的 `原因:` 行与
`odom.rejected`/`tf.last_lookup_delay_s` 一并回传。
