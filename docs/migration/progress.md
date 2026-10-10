# 进展

当前状态与最近变更。逐轮流水（S0–S3、Codex 各轮复审、修复过程）已移入
[进展历史](../archive/progress_history.md)，失败证据与“用户验证/本轮实测”区别原样保留。

## 2026-10-10（第二轮）：运行路径全 C++ 化（进行中）

用户决定：**不再需要 Python 实现**，运行路径全部改为 C++。

- 新增 ament_cmake 包 `src/sentry_scan_adapter_cpp/`，端口 5 个节点：
  `rm_input_adapter`（PCL 读云，原生支持现场 48 字节 PCL 布局）、`cmd_gate`、
  `task_adapter`、`shadow_guard`、`input_pause_gate`；launch/config/rviz 一并迁入。
- **删除** Python 包 `src/sentry_scan_adapter/`（含其 3 个 Python 单测脚本）。
- 命令闸门互锁矩阵改为 C++ gtest：`build/sentry_scan_adapter_cpp/test_cmd_gate_logic` **11/11 通过**
  （默认不输出/需授权/限幅/候选超时/不健康/地图过期/NaN/取消清除使能/允许 wz）。
- C++ 适配层已单独验证：`cpp_livox_cloud`（现场真实 48 字节布局）健康 + 候选速度 1.00 m/s。
- **未决**：全 C++ 栈下 `mode1_goal` 健康为真但无运动 —— fake 已发出 `/sentry_scan/task/goal_in`、
  订阅计数也匹配，但 C++ `task_adapter` 未收到该消息（无任何拒绝日志）。需再排查
  （QoS/discovery/话题解析）。因此**纯 C++ 全链路尚未验收**，36 场景矩阵也未在 C++ 栈上重跑。

## 2026-10-10：现场第二次故障（云被拒 + RViz 空白）修复

- 现场 `start_rviz:=true` 后 RViz 空白，适配器持续 `cloud rejected: PointFields and structured
  NumPy array dtype do not match`：真实 `/cloud_registered` 是 **PCL 风格 48 字节布局**
  （`x@0 y@4 z@8 normal_*@16..24 intensity@32 curvature@36`，字段间有空洞），
  `do_transform_cloud()` 无法处理。
  → 改为适配层自己做变换并输出**只含 xyz 的密集云**（`point_step=12`）；单测覆盖真实 48 字节布局、
  行填充、大端/FLOAT64/截断拒绝；新增场景 `livox_cloud`（健康+运动、无云拒绝）。
- RViz 空白根因：`setup.py` 只安装 `*.launch.py`/`*.yaml`，**`.rviz` 从未安装**，而 launch 用
  share 下该文件启动 RViz → 空显示列表。
  → `setup.py` 安装 `launch/*.rviz`；launch 缺失时 `[WARN]`；`run_shadow_onsite.sh` 预检报错；
  新增 `rviz_config` 回归场景。
- 本轮仍是只读影子；未启动实车节点、未发布底盘命令。

## 2026-10-09（第二轮）：实车采集根因定位与修复

- 在本机直接读原始采集目录后定位 `health_ok=False` 根因：RM `TfTransformer::odom_callback`
  **先发 `/Odometry_transformed`、后广播同 stamp 的 `odom→base_link`**，而适配器按消息时刻查询该动态 TF，
  只能回退到上一周期样本（≈0.1 s）> 当时的 `tf_future_tolerance=0.05` → odom 每帧被拒。
- 修复（仅本仓库）：`odom_in_planning_frame`（默认 true）下直接用消息位姿、参考点差异用**静态** TF 修正；
  `tf_future_tolerance` 按实测周期改为 0.15 s；新增位姿复合单测（adapter_math 14 项通过）。
- 新增可复现场景：`odom_before_tf`（修复后健康+有速度、无 TF 拒绝）与
  `odom_before_tf_legacy`（旧行为 `health_ok=False`、输出全零、日志出现 `needs future data`）。
- 用户确认：TF 无 `world` 系（按不存在处理，速度保持降级为零）、SCAN 在 `odom` 规划、
  IMU 取自 mid360 且雷达≈机体中心、允许第 7 项（`uart` 的 `twist_topic` 指向闸门输出）。
- 仍 UNKNOWN：车辆几何中心与 `base_link` 的物理关系、碰撞包络实测尺寸、急停与 MCU 看门狗、
  修复后现场 `health_ok` 是否转 true（需带修复重跑影子）。
- 用户确认实车包络：**半径 0.26 m、高 0.15 m 的圆柱**（→ `robot_height=0.15`、`body_height=0.075`、
  `safety_margin=0`）；现场**存在可立即停车的急停**。
  `run_shadow_onsite.sh` 默认值已按实车包络更新，并用这两个值做了端到端本地验证
  （假输入下预检通过、`health_ok=True`、候选速度 1.00 m/s、图隔离 PASS）。
  仍未确认：MCU 速度命令超时归零时限。
- 证据口径：修复后的完整矩阵 `final8` 跑到 **11/30** 时因现场机器断电中断（11 个全过），
  另补跑 `mode1_goal`、`task_frame_transform` 通过 → 修复后已确认 **13/30**；剩余 17 个场景**待重跑**
  （用户要求长任务先暂停，稍后再跑）。此前 `final7` 的 28/28 对应**修复前**代码。

## 2026-10-09：首次实车采集资料离线分析

- 用户提供 `collect_20261009_205335`，分析见[现场采集报告](../testing/onsite_field_2026-10-09.md)。
- 已有真实输入证据：主要话题/TF 可见；`world` TF 未找到；TF 定义中 base_link 与 lidar_link 重合，机体几何中心仍待测量确认。
- 本批没有影子节点/健康成功记录，控制审计与主采集报告矛盾；只能算输入摸底，不能算 I2 影子验收通过。
- 本轮仅写文档，未构建、未启动 ROS、未操作实车、未修改导航参数或参考仓库。下文早期“未接实车”等状态须按各轮日期理解；当前仍未授权底盘接管。

## 2026-10-07：交接 A 仓库整理完成；交接 B 待批准

用户决定暂停扩建仿真、恢复实车替换主线，并指定“先完成任务一，暂停汇报，批准后再做任务二”。

**本仓库整理（交接 A，已完成，本轮执行）**

- 历史资料移入 [docs/archive/](../archive/README.md)（13 份过程/证据页 + 进展历史），
  归档页首写清被哪份当前文档取代；运行资产 `docs/testing/maps/`、`docs/testing/evidence/` 原路径不动。
- 当前入口收敛为 [docs/README.md](../README.md) 一处；`progress.md`/`plan.md`/`decisions.md` 去重复叙述。
- 纠正陈旧表述：`interface_mapping.md` 的“所有 adapter 未实现/无执行端”、
  `inflation_analysis.md` 写反的 z 膨胀方向、`progress.md` 底部“地面分割尚未实现”等历史挂载。
- 登记脚本分组见 [scripts/README.md](../../scripts/README.md)；未搬迁代码、未改导航行为。
- 映射、保留边界与验证见[整理报告](cleanup_report.md)；基线快照见
  [cleanup_baseline_2026-10-07.txt](../testing/evidence/cleanup_baseline_2026-10-07.txt)（基线提交 `e35877b`）。
- **本轮未构建、未启动 ROS、未跑场景、未接实车**；上面的仿真结论都来自此前轮次，不是本轮复跑。
- 审查（对照 `37b22ac`）指出的 2 处文档错误与 2 处小问题已修正，见[整理报告](cleanup_report.md)第九节；同样只改文档。

**交接 B（下一开发任务，未开始）**

- [实车影子接入交接](real_robot_handoff.md)：I1 数据契约 → I2 adapter/影子 launch/候选速度/健康门控 →
  回放与影子验收。本轮 Codex 只交付计划，未实现任何 adapter，未授权硬件或底盘输出。
- 参考仓库 RM main `7dfe71a`、SCAN2 main `103bce4` 状态未变；RM 两项未跟踪内容与 SCAN1 原有修改保留。

## 2026-10-09：改为实车在线只读影子；现场工具与接管方案交付

用户决定：没有 ROS bag，改为**实车在线只读影子验证**；暂停扩建仿真与 Gazebo。
本轮授权读取实车输入、实现影子接入；**不授权发布真实底盘命令**（受控驱车需 Codex 审查 + 用户同意）。

- 上一轮两项审查问题确认已修：行填充点云先重排再变换（`rm_input_adapter.py:_densify_cloud`，
  测试 `test_padded_cloud_survives_check_densify_and_transform`）；地图过期始终停车、
  删除 `revoke_on_map_stale`（`shadow_guard.py:decide`，测试 `test_stale_map_always_stops_*`）。
- 新增现场只读工具（**本机实测**，合成输入下彩排通过，场景 `onsite_tools`）：
  `onsite_inspect.py`（话题/类型/QoS/频率/时间戳/TF/控制发布者 → `log/onsite/<ts>/report.{txt,json}`）、
  `run_shadow_onsite.sh`（预检 + 启动影子）、`onsite_send_goal.py`（只发 `/sentry_scan/task/*`）、
  `onsite_check_safety.sh`（影子未接真实控制入口）、`onsite_record.sh`（可选录包）。
- 文档：[在线影子运行手册](onsite_shadow_runbook.md)（现场最短流程、采集命令、回传清单、7 个必答问题）、
  [实车接管方案](real_robot_takeover_plan.md)（PROPOSED：单一速度源、默认撤权、急停与退回、首轮低速步骤、
  默认不改 RM）。
- **待现场确认**：实际话题/类型/QoS/频率、TF 链、odom 参考点与速度坐标系、`world`↔规划系关系、
  真实云 stamp 与时钟尺度、底盘控制发布者名单；以及手册 §8 的 7 个问题。
- **四轮复审 4 项收尾（本机实测）**：安全自检改为按 (命名空间, 节点名) 判定，实车 `/uart_node`
  不再被误判（场景 `external_uart_coexist`）；采集器在观察期内**持续发现**话题、`/tf_static` 与 `/map`
  用 transient_local（场景 `onsite_late_inputs` + `check_onsite_report.py`）；
  新增**影子专用可暂停输入闸门**用于运行中断流/恢复验证（场景 `input_pause_gate`：
  实测停车延迟 0.392 s、恢复后保持零、新任务后才恢复运动）；`onsite_record.sh` 校验
  `metadata.yaml` 与消息数，异常返回非零（实测 1637 条消息正常结束）。

## 2026-10-07：交接 B 的 B1/B2 实现完成，B3 影子验收（合成输入）

用户批准开始任务二。**本轮实现了影子接入代码**，仍未接实车、仍未向底盘输出。

- 新增 `src/sentry_scan_adapter`（薄适配包）与 `/sentry_scan` 影子入口：
  `rm_input_adapter`（TF/odom/云/速度适配，时间/有限值/TF/定位跳变门控，失效发布 `planning/reset`）、
  `shadow_guard`（只发 `cmd_vel_shadow`；周期检查 `/cmd_vel` 发布者；yaw/linear.z 恒零）、
  `check_inputs`（输入健康与配对检查工具）。
- SCAN 最小改动：GridMap 可选严格 sensor/cloud 配对（仿真默认不变）、跟踪器
  `yaw_candidate_enabled`、可视化 frame 可配置（默认保持仿真硬编码）。
- 契约：[影子输入契约](../interfaces/shadow_input_contract.md)；参数模板
  `src/sentry_scan_adapter/config/shadow_contract.yaml`。
- 验收：[影子验收证据](../testing/shadow_acceptance.md)。判据脚本 `scripts/check_shadow_graph.py`、
  场景编排 `scripts/test_shadow_entry.sh`、合成输入 `scripts/fake_rm_inputs.py`、
  数学单测 `scripts/test_shadow_adapter_math.py`；日志与影子 CSV 在 `log/shadow/`（不入库）。
- **已验证**：启动隔离（无 UART/Nav2/模拟器、无 `/cmd_vel` 发布者）、三模式产生候选速度且 yaw 恒零、
  云/odom/TF 中断与旧 stamp 重发/时间倒退/定位跳变均失效锁止、取消后不重新运动、
  GridMap 严格配对拒绝、坐标/杆臂/yaw=90° 数值样例。
- **审查修正（对照 `54a1b1d`–`c1a2440` 的 5 项）**：新增 `task_adapter` 做目标/路线坐标变换
  （空/未知 frame 明确拒绝）；点云按结构+有效点校验（全 NaN 拒绝）；GridMap 发布
  `grid_map/cloud_update` 心跳并由 guard 以 `max_map_age` 门控；guard 只禁止**影子命名空间内**
  节点发布真实控制话题（允许与 Nav2 并存）；停车判据改为"先动后停+限时归零+整窗为零+恢复不复活"
  （新增 `check_shadow_stop.py`）；新增未来时间戳容差 `max_future_stamp`。
  合成场景矩阵 **20/20 通过**（`log/shadow/final2_matrix.log`）。
- **第二轮复审（对照 `34519cc`）3 项收尾**：地图心跳停更现在会**撤销任务并锁止**（`latched_task_id`，
  仅编号更大的新任务+地图恢复才解除，心跳恢复本身不放行旧速度）；停车判据补齐观察窗首尾覆盖、
  样本有限值，并改用注入方发布的 `test/fault_marker` 按**实际事件时刻**计时；
  点云检查改为按 PointCloud2 布局解析（只接受小端 FLOAT32，行填充按行首址寻址，其他布局显式拒绝）。
  合成场景矩阵扩到 **22 个**（新增 `map_relatch`、`map_relatch_newtask`）。
- **第三轮复审（对照 `1786c60`）2 项收尾**：带行填充的点云在 TF 变换前按行首址**重排为密集布局**
  （`tf2_sensor_msgs.do_transform_cloud` 会按连续 `point_step` 遍历，读端沿用输入布局即误读）；
  删除 `revoke_on_map_stale` 开关——地图过期**始终**输出零并撤销+锁止，不允许配置成"过期仍放行"。
  新增 `guard_logic` 门控逻辑单测（6 项）与 `padded_cloud` 全链场景，矩阵扩到 **24 个**。
- **未完成/未验证**：真实 RM 数据或录包回放（I2 真实影子验收）、`world` 与 `odom` 的数值关系、
  IMU 杆臂与角速度、真实云 stamp 与本机时钟尺度、真实洞净高与实车最低包络、实车任何操作。

## 2026-10-06：Mode 2 洞口—坡道往返与无支撑面六航点预览

两个实验都是**无界面运行（CONFIRMED，当轮实测）**；GUI 未操作，RViz 图形交互仍未验收。

- [有支撑面往返](../archive/mode2_field_roundtrip.md)（已归档）：两个稀疏航点完成洞外→穿洞→上坡→
  折返→下坡→出洞，行程 5.808 m，升/降各 0.200 m，终点误差 0.00497 m，实际障碍云碰撞 0/6202 帧。
  首版 6 个密集航点动态可行性失败后越界，失败证据保留。
- [无支撑面六 XYZ 航点预览](../testing/mode2_waypoint_z_preview.md)：原始 PCD、无地面网格，
  复用开环样条 odom；六航点顺序进入 0.11 m 邻域，升/降约 0.203 m，终点 XY 误差 0.00405 m，
  原始 PCD 全点包络检查 0/4200 帧。模型 R=0.26/H=0.10、平地底部离地约 0.10 m，
  **不是贴地爬坡或速度闭环结论**；旧阈值试跑 43 帧碰撞，未称通过。

## 2026-10-02：指定真实 PCD 坡道与洞口闭环

用户批准有界支撑层选择。详见[真实 PCD 路线修复](../testing/pcd_route_fix.md)。

- **CONFIRMED（当轮实测）**：修复 A* 负坐标向零取整（改 `floor`）与模拟雷达虚增洞顶厚度
  （`preserve_map_geometry`）；新增指定入口/方向/边界的连续支撑层选择。
- 大坡 H=0.25 m 到达（升 0.200 m，误差 0.00081 m，碰撞 0/2633 帧）；两洞 H=0.10 m 到达
  （升 0.160/0.200 m，碰撞 0/2434、0/2432 帧）；同两洞 H=0.25 m 拒绝、全程无非零命令。
- 局部净高约 0.24 m（点云约 4 cm 量化）；真实洞净高与实车最低包络仍 **UNKNOWN**。
- 9 包构建、6 项地形单测与 CTest 通过；未重跑历史全场景矩阵。接口/TF 不变，参考仓库只读。

## 当前边界与未验证

- **未接实车**；所有输出都在 `/sentry_sim`（或实验用的 `waypoint_z_preview`）命名空间内，
  不存在通往 UART/底盘的路径。
- **RViz 图形交互本环境无法验证**（创建不了 OpenGL 上下文）；三条地形路线由用户确认，
  六航点最新预览只有无界面测试，不能统称 GUI 已验收。
- 无轮地接触/牵引/打滑动力学；A* 搜索高度仍含起终点线性参考；无全场多层地形图。
- 真实尺寸与诊断尺寸必须区分：R=0.26/H=0.25 是用户给定基线，H=0.10 与离地预览是诊断条件。
- 参考仓库无受跟踪源码修改；`log/`、`artifacts/`、`build/`、`install/` 不入库，本地保留供追溯。

## 下一步

1. 用户批准后执行交接 B 的 B1（输入契约 + 影子骨架）与 B2（接通 SCAN、三模式、候选速度），
   到 B3（回放/影子验收）停止，交 Codex 审查。
2. 实车操作（场地/运行许可）与 I3 驱车另行授权；不得把影子结果写成实车验收。
