# DeepSeek 交接 B：I1/I2 实车数据适配与影子运行

日期：2026-10-07。**B1/B2 已实现，B3 合成输入验收已完成**；真实数据影子验收与 I3 未开始。

> 实现与证据：[影子输入契约](../interfaces/shadow_input_contract.md)、
> [影子验收证据](../testing/shadow_acceptance.md)、`src/sentry_scan_adapter/`。
> 审查（对照 `54a1b1d`–`c1a2440`）提出的 5 项已修正：目标/路线坐标适配、点云有效性与
> 地图更新门控、与 Nav2 并存的保护层语义、先动后停的停车判据、未来时间戳拒绝。
> 第二轮复审（对照 `34519cc`）的 3 项也已收尾：地图停更改为**撤销+锁止**（新任务才解除）、
> 停车判据按 `test/fault_marker` 的实际事件时刻计时并检查观察窗首尾覆盖、
> 点云按 PointCloud2 布局解析（不支持的布局显式拒绝）。
> 第三轮复审（对照 `1786c60`）的 2 项也已收尾：行填充点云先重排为密集布局再变换；
> 地图过期始终停车并撤销+锁止（删除可关闭该行为的 `revoke_on_map_stale`）。
> 本页以下内容是任务要求与边界，仍然是评审依据；其中"未实现 adapter"等表述指当时状态。用户已决定暂停扩建仿真、恢复 C 方案实车替换工作。
本轮 Codex 交付实现计划，未实现以下 adapter，未授权启动硬件或向底盘输出。
具体包名、frame 与新增接口是 **PROPOSED**；C 方向及推进实车接入是用户已确认的目标。

## 一句话目标

**把模拟定位和模拟雷达换成 RM 的真实数据，让 SCAN 输出正确的轨迹和候选速度，
但暂时不接串口、不驱动车辆。** 先得到可审查的实车软件接入版本，再单独安排低速实车。

不要再搭 Gazebo，不重做已完成的全向跟踪器，不一口气替换 Nav2 的 BT/actions/恢复职责。
RM 原感知定位与 Nav2 保留作为基线和回退。实现仍在本工作区，`../VI_26_Sentry` main 只读；
这一步通过独立 launch/overlay 接入 RM，不是去参考目录直接删换源码。
将来确需修改部署仓库，先由用户指定可写集成分支/工作区，另行交付最小补丁。

## 现状与证据边界

本轮静态核对：RM main `7dfe71a`，SCAN2 main `103bce4`；本仓库 HEAD `5bbc87b`
加大量未提交实施成果，不能只按 HEAD 判断能力。开始前按[清理交接](cleanup_handoff.md)保存基线。

| 已有，可复用 | 仍需实现/核验 |
|---|---|
| 三种任务模式、全向 XY 跟踪、odom yaw、圆柱包络、若干取消/断流保护 | RM 数据坐标/时间/参考点适配、真实输入健康检测 |
| 速度闭环仿真与回归脚本 | 不含任何模拟 odom/雷达的独立影子入口 |
| Mode 2 六 XYZ 点无支撑面预览 | 真实地面/洞顶点云、实车高度和坡面接触是否匹配 |
| 用户验证 RM 原 Nav2 导航可用 | SCAN 候选命令坐标、UART/MCU 停车与授权机制 |

六航点结果只证明给定诊断尺寸和离地间隙下存在可执行的空间样条；不是实车爬坡证明。
实车 z 必须来自真实定位。禁止把 `.17/.37` 预览高度、H=.10、原始图平移/切高参数直接带上车。

## 工作拆成三个小交付

### B1：冻结输入契约与实车影子骨架

建议新增一个薄适配包 `src/sentry_scan_adapter/`，不要复制 SCAN 核心。
提供独立 `sentry_scan_shadow.launch.py`，复用本仓库 planner 和 closed-loop tracker。
默认只读输入，输出全部隔离到 `/sentry_scan/`；该 launch **没有真实命令输出开关**。
不启动 UART、Nav2、LIO、registration 或机器人；它们由已有部署入口管理。

输入候选及必须核对的源码：

| 输入 | 证据入口 | 必须处理 |
|---|---|---|
| `/Odometry`、`/Odometry_transformed` | RM `hnurm_fastlivo2/FAST-LIVO2/src/LIVMapper.cpp::publish_odometry`；`hnurm_bringup/src/tf_transformer_node.cpp` | 主 odom 未填有效 twist；参考点和左右乘不能只看 frame 名 |
| `/LIVO2/imu_propagate` | `LIVMapper.cpp::imu_prop_callback` | pose + 世界系 `vel_end`，header world；须核实与 camera_init/odom 的实际坐标关系及 IMU/车体参考点 |
| `/cloud_registered` | `LIVMapper.cpp::publish_frame_world` | camera_init 云，现有 stamp 是发布时间，不伪称硬件采样同步 |
| `/segmentation/obstacle` | RM 地面分割节点 | 可作为对照输入，但先检查低障碍/洞顶/坡面分类效果 |
| `/tf`、`/tf_static`、外参 | `hnurm_bringup/params/extrinsic.yaml`、TF/registration 源码 | 用消息时刻变换；不能补未经证明的 world=odom 单位 TF |

建议局部规划用 `odom`、任务/路线用 `map`，但先写出矩阵与来源再冻结。
adapter 输出车体规划参考中心，射线原点单独取真实雷达中心；两者不能互换。
记录 `T_planning_body`、`T_planning_sensor`、点云变换及速度旋转，包含参考点杆臂影响；
若需要角速度而输入没有，显式缺失/降级，不能悄悄把 IMU 点速度当车体中心速度。

**B1 输出**：实际输入契约、参数模板、可启动且失效关闭的骨架、输入检查工具。
缺实车/rosbag 时继续做录制说明和消息级测试，不制造“真实输入已通过”的结论。

### B2：接通 SCAN、三种模式和影子候选速度

1. adapter 将有效云、机身 pose/世界系速度、传感器 pose 提供给 SCAN。
   SCAN 已使用规划系点云时才设 `cloud_is_world=true`、`need_extrinsic=false`，杜绝外参重复作用。
2. **云和射线原点必须按时间配对**。当前 `GridMap::sensorPoseCallback/cloudCallback`
   分别缓存最新数据，仅先发布 pose 再发布 cloud 不能保证一致。
   记录配对偏差，限制等待/插值范围，过期/缺失直接拒绝。
   若必须改 GridMap，做可选、最小的同步输入模式，保留仿真默认行为并测试乱序；
   涉及新增消息或较大架构变化先给具体接口提案，不静默扩大。
3. 保留原始点云用于诊断；真实规划使用哪种云必须明确。不得盲用高度截断后的 `/pointcloud`，
   也不得保证原始云必然可贴地通行。允许对照已有地面分割输出，不新建全场地形算法。
   若地面被判障碍，交付可复现证据并保持拒绝，不能移高机器人或删低障碍换取通过。
4. 所有规划、目标、路径、包络和 RViz 在选定 frame 下保持一致；
   修正 `PlanningVisualization` 的硬编码 world/map 为可配置 frame（保留仿真默认），
   不靠改消息 header 假装完成变换。Bspline 无 frame，契约固定为该规划系。
5. Mode 1：RViz 目标按 TF 转规划系；保留原高度语义，先用于已知平地区域。
   Mode 2：多个 **机体参考点 XYZ**；任务系路线在就绪后转换，避免初始无 TF 时自动误执行。
   Mode 3：地面参考 Path，加 body_height 一次。不得将 Mode 2 中心 z 当成 Mode 3 地面 z。
   map→odom 重定位跳变时撤销旧任务/轨迹，显式重新接受任务；不继续旧坐标轨迹。
6. 复用 `closed_loop_controller`，输出 remap 到候选速度，**绝不启动 open_loop_controller**。
   当前跟踪器按实际 yaw 把世界 XY 速度转为 body XY；实车适配需核实该 body 与电控命令系关系。
   当前 `yaw_mode=hold` 仍会产生保持初始朝向的 angular.z，不等于“不接管朝向”。
   影子版明确隔离/置零 yaw 候选输出，保留 MCU 所有权，不将 wz 偷塞 linear.z。
7. 实现输入健康门控：消息源 stamp 与本机接收年龄都检查；有限值、四元数、TF、
   点云中断、时间倒退、定位跳变均可观测。当前 follower 的 odom 超时按接收时刻计算，
   旧 stamp 消息不断重发不应绕过检测。拒绝新输入时不能仍把缓存旧云宣告健康。
8. 失效后撤销任务并锁止候选输出；恢复输入不自动恢复旧轨迹，需新任务。
   记录 planner/follower 重启、任务编号回退、延迟授权、旧轨迹跨新任务的行为；
   现有 TaskAuthorization 与 Bspline 没有共同任务字段，不能把已有 cancel 测试当成完整跨进程安全证明。
   影子模式可先要求整组重启并清空任务，不能靠定时自动重授权掩盖问题。

**B2 输出**：影子 launch、adapter、参数、RViz、确定的接口表、必要的最小 SCAN 改动及测试。

### B3：回放/影子验收，交 Codex 审查后停下

有录包时用真实 RM 数据回放；没有录包时用合成消息验证契约，但标记“真实输入待测”。
回放用独立 ROS_DOMAIN_ID、`/clock` 与一致 `use_sim_time`，并重映射录包中的
`/cmd_vel`、`/cmd_vel_remap` 等控制话题到隔离名称，禁止把录包命令发到车辆。
在线只读影子观察也需用户安排场地/运行许可，不自动启动硬件。

最低测试，不需要再搭仿真世界：

| 测试 | 必须观察到什么 |
|---|---|
| 启动隔离 | 无模拟 odom/雷达、Go2 模型、UART；新增节点不发布原 `/cmd_vel`、`/cmd_vel_remap` 或原定位话题 |
| frame/杆臂/速度样例 | 非零平移 + yaw=90° 的已知样例数值正确；缺 TF 拒绝，不以改名通过 |
| 时间/QoS | 实际端点可连接；旧 stamp 连续重发、乱序 pose/cloud、缺配对、时间回退被检测 |
| 三模式 | 正确 frame/z 输入产生任务/轨迹；未就绪不偷偷执行；错误 frame 明确拒绝 |
| 输入失效 | odom/cloud/TF 分别失效，候选速度归零并锁止；记录事件时刻、停止延迟、完整持续观察窗 |
| 任务时序 | 取消→延迟旧授权/轨迹→新任务，以及各进程重启；旧任务不重新激活 |
| 重定位 | map→odom 跳变使旧任务失效；无轨迹坐标突跳后继续输出 |
| 碰撞/高度 | 使用真实尺寸参数；低障碍/洞顶不被过滤掉；不把规划受阻改写成“接入成功即可通行” |

检查已有仿真关键回归即可：Mode1/2/3、横移、取消竞态、odom 断流、gap_edge、A* 上下坡索引；
仅当修改地图/高度路径时增加相关地形回归，不重建更多场景。
每个结果带命令、配置、数据标识、测量与 PASS/FAIL/NOT_RUN；断言失败非零退出。
真实数据缺失允许交付代码和合成测试，但 **I2 真实影子验收仍未完成**。

## 拟新增接口（开发前冻结，完成后更新 interfaces）

以下是建议，不是当前已存在端点；禁止只把旧仿真 `/sentry_sim` 整体改名前缀就声称适配完成。

| 接口 | 类型 / 流向 | frame、时间、QoS、频率建议 |
|---|---|---|
| `/sentry_scan/body_pose` | Odometry：adapter → FSM/GridMap/follower | 规划系机体中心，twist 明确为规划系；来源 stamp；SensorDataQoS，随有效 odom |
| `/sentry_scan/sensor_pose` | Odometry：adapter → GridMap | 同一规划系雷达射线原点；与云配对时间；SensorDataQoS，随配对云 |
| `/sentry_scan/cloud` | PointCloud2：adapter → GridMap | 真正转换到规划系；保留来源 stamp，注明发布时戳限制；SensorDataQoS，随有效云 |
| `/sentry_scan/move_base_simple/goal` | PoseStamped：任务 adapter → FSM | 已转换规划系，保留输入 stamp/转换时刻契约；Reliable/Volatile/1，事件 |
| `/sentry_scan/initial_path` | Path：路线 adapter → FSM | 规划系地面 z；Ready 后发送；Reliable/Volatile/1，事件 |
| `/sentry_scan/planning/bspline` | Bspline：FSM → follower | 固定规划系，原 start_time；Reliable/Volatile/10，规划事件 |
| `/sentry_scan/cmd_vel_candidate` | Twist：follower → 影子保护/记录 | 明确 body XY、不是 UART 命令；接收时刻；Reliable/Volatile/20，100 Hz 目标频率 |
| `/sentry_scan/cmd_vel_shadow` | Twist：影子保护 → 记录/显示 | 已门控候选值，仍不接车；接收时刻；Reliable/Volatile/1，配置频率并实测 |

健康状态/故障原因可以用标准 diagnostics；依赖先检查，不为一个状态位引入庞大框架。
reset/task_active 等沿用已有类型和语义并记录新增消费者；若改任务协议先列兼容影响。
对所有新增 TF（若确有必要）登记 parent/child/publisher/pose source/频率，保证无重复发布者；
不复制仿真 world→base/sensor 到实车树。滑动地图子 frame 使用独立名称避免冲突。

## 下一轮之前需要用户/电控提供什么

不必等齐才写 B1/B2，但缺这些不能通过对应实车门槛：

1. 现车实际启动命令、参数覆盖和 overlay 顺序；一段含 odom、传播 odom、原始云、TF 的短录包。
2. 核对已提供 R=.26/H=.25 与雷达外参是否仍对应当前车，确认参考中心/旋转轴/机械最低点。
3. 电控 vel_x/vel_y 的坐标系、正方向、单位；哪些旋转策略真正运行。
4. 固件命令断流停车时限、急停与人工接管方式，以及首轮平地速度/加速度上限。

## 何时才开始替换实车速度源（I3，不属于此次实现）

影子审查通过、轴向/单位/真实尺寸确定、MCU watchdog 与急停验证后，另开受控实车任务：
独立命令保护层、默认撤权、单一速度源、所有 Nav2 controller/behavior/决策旁路的互斥切换、
限速限加速度、故障立即停止、可恢复到原 Nav2。
不能只加一个 mux 而保留旧节点直发 UART；保护节点自身死亡需 MCU 或独立 watchdog 兜底。
先平地低速直行/横移/停止，再已知坡面；真实高度未通过的洞口不得用缩小包络试车。
最后 I4 才迁移 BT/actions/任务结果与 footprint 依赖，不提前删除 Nav2。

## 交付给审查者

分别提交：基线/契约、adapter 与影子入口、测试与操作说明；不混入文档搬迁大 diff。
给出真实命令，不提前编造尚不存在 launch 的“成功运行记录”。更新 plan/progress/interface 表，
报告缺失数据和未解决风险。交付到 B3 即停止，等待 Codex 审查和用户实车运行许可。
