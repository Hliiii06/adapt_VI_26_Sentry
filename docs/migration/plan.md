# C 方案实施计划：先独立全向仿真，再接实车

用户最新顺序：**SCAN 三种模式保留 → 全向运动与碰撞适配 → PCD/RViz 闭环仿真 → 审查 → 接入 VI_26_Sentry**。这替代先做 RM 输入适配的旧 P0–P5 顺序。

本轮只更新文档。用户将交由 DeepSeek harness 实现代码与仿真，随后由 Codex 审查；尚未执行或验证实现。实施入口见 [harness 交接说明](implementation_handoff.md)。

## 第一阶段：独立 SCAN，不依赖实车

| 阶段 | 通俗说明与交付 | 完成条件 |
|---|---|---|
| S0 固定起点 | 记录 SCAN 提交/已有修改、实施目录；确定机器人尺寸、PCD 路径/单位/坐标、初始位置和仿真参数 | 不覆盖用户文件；缺 PCD 时可用明确标注的合成地图先开发，不宣称用户地图验收通过 |
| S1 全向适配 | 读 odom 姿态，分开机身朝向与运动方向；取消强制先转头再走；修改相关碰撞、跟踪、FSM 时间处理 | 能保持车头方向横移；朝向变化不改变预期世界系运动方向；三种模式入口保留 |
| S2 PCD + RViz 闭环 | PCD 模拟周围雷达观测，规划后输出速度，运动模拟器积分得到 odom；配置 RViz 与统一启动入口 | 用户可以启动、选模式、看机器人根据速度运动；不是把轨迹直接抄成 odom |
| S3 场景验证与交接 | 三种模式、固定朝向横移、边转边走、窄通道、失效停车；输出日志、操作说明、改动清单 | 按验证表记录通过/失败/未测；交给 Codex 审查，修正问题后再进入实车接入 |

首个可交付版本到 S3，不要求先接 UART、迁移 BT 或实现 Nav2 actions。仿真先使用固定 yaw 与可配置旋转模式；旋转仅用于测试跟踪和包络，不声称复刻电控小陀螺。

### 保留三种输入

- Mode 1：RViz “2D Goal Pose”指定目标；保留原高度语义，不承诺目标箭头控制实际车头。
- Mode 2：参数配置多个航点；不得在重写 launch 时遗漏此模式。
- Mode 3：输入带高度的参考路线；地面 z 与 body_height 只相加一次。

Mode 1 参考生成不是地图全局通路搜索，不要求复杂迷宫可达性等同 Nav2。Mode 2/3 不能只验证启动成功，还要记录路线实际执行情况。

### 碰撞检查怎么改

运动方向不等于车头方向，不能继续无条件以轨迹切线代表车体 yaw。当前 yaw 来自有效 odom，但不能把这个角度当作整条未来轨迹的真实姿态。

建议首版用覆盖所有旋转角度的保守包络（具体模型/参数仍属 PROPOSED），适合未来朝向未知的旋转场景；之后可选已知朝向预测的精细模型。明确长度、宽度、高度、参考中心与安全余量，不只是放大二维半径。

全向适配涉及的碰撞查询、优化碰撞代价、FSM 安全检查及 RViz 包络应保持一致。只修改显示或一个检查入口不算完成。允许针对这些依赖做必要的局部代码修改，不要求保持 SCAN 核心逐行不变；不进行无关重构或自由三维算法重写。

### 仿真是如何闭环的

```text
PCD + 模拟位姿 → 局部雷达观测 → SCAN 地图/规划 → 全向跟踪速度
          ↑                                      ↓
          └──────── 速度积分产生模拟 odom ──────────┘
```

RViz 是显示和目标输入工具，不是运动模拟器。优先复用已有 PCD 渲染和模拟组件，再局部适配全向运动。模拟器按 body vx/vy 和 yaw rate 积分，向 SCAN 提供其需要的规划系速度与姿态；不能将样条位置直接写进 odom 来伪造跟踪成功。

首版平地固定 z；带高度场景必须说明采用参考高度跟随还是地面采样等简化规则，单独标注结果，不宣称验证真实爬坡动力学。PCD 不存在、初始点无效或外形参数无效时清晰报错。

仿真可用 world 作为统一坐标系；PCD、传感器原点、odom、轨迹和 RViz Fixed Frame 必须一致。未来接车才评估 odom/map，不为复用名称增加虚假单位 TF。

## 第二阶段：审查后接入 VI_26_Sentry

| 阶段 | 工作 | 门槛 |
|---|---|---|
| I1 接口核对 | 记录已验证 RM 启动/overlay；用真实点云/odom 替换模拟输入；确认命令坐标、时间和速度来源 | 保留原 LIO/registration/TF/协议，不以静态疑点先重构旧链 |
| I2 影子接入 | 接收实车数据但不输出底盘命令，比较规划与姿态/云 | 坐标/时间正确、数据失效可检测；实车操作另获授权 |
| I3 受控实车 | 切断旧速度源后低速闭环，保留 Nav2 回退 | 明确单一控制源、MCU 失联停车与急停；获用户测试许可 |
| I4 决策迁移 | BT actions、任务反馈/恢复、footprint 依赖及 Nav2 职责清理 | 单独确认范围，不作为 S1–S3 前置 |

保留下面较详细的实车接口草案供第二阶段使用；独立仿真不必先创建整套 RM adapter/action facade。包名与落盘位置由 harness 在实施前记录，优先复用 SCAN 现有包，不为遵循旧目录建议机械拆包。

## 后续 I1–I3 实车接入草案（不是独立仿真的前置）

下列名称均为拟新增接口，不是当前已存在端点。所有 SCAN 内部名称 remap 到隔离命名空间。

| 拟定 topic / type | publisher → subscriber | frame / 时间 | QoS / 频率草案 |
|---|---|---|---|
| /sentry_scan/body_pose · Odometry | 输入 adapter → FSM/地图/follower | 规划系 odom，明确机身参考点；速度按 SCAN 契约为规划系；保留来源 stamp | SensorDataQoS；随有效里程计，实测频率待记录 |
| /sentry_scan/sensor_pose · Odometry | 输入 adapter → GridMap | odom 中真实射线原点；与云配对时刻 | SensorDataQoS；随云配对 |
| /sentry_scan/cloud · PointCloud2 | 输入 adapter → GridMap | 真正变换到 odom，保留云 stamp | SensorDataQoS；随云，禁止伪造同步 |
| /sentry_scan/goal · PoseStamped | 目标 adapter → FSM Mode 1 | map/RViz 目标经 TF 转 odom；有效输入后发送 | reliable/volatile depth 1；事件 |
| /sentry_scan/initial_path · Path | 路线 adapter → FSM Mode 3 | odom，地面参考 z；body_height 由 SCAN 加一次 | reliable/volatile depth 1；确认就绪后事件发布，明确重发规则 |
| /sentry_scan/planning/bspline · Bspline | FSM → follower | 消息无 frame；契约固定 odom；校验 start_time/id/knots | reliable depth 10；规划事件 |
| /sentry_scan/cmd_vel_candidate · Twist | follower → 保护层 | 经核对的底盘命令系；无 header，保护层记录接收时间 | reliable depth 1；建议 100 Hz，待负载验证 |
| /cmd_vel · Twist | 唯一授权输出 → 现有 UART | 保留既有协议坐标语义；不重用 z 作垂直运动 | 兼容 UART reliable；仅实车授权模式发布 |

实车规划系 odom 是当前建议而非已冻结事实。若 I1 发现其与 LIO 状态的原点/轴关系不清晰，先完成显式变换设计，不增加一个未经证明的 world=odom 静态 TF。高频 odom 的 world header 不证明已有该 TF；速度需旋转及必要的参考点补偿。

原始 /cloud_registered 优先于已高度截断的 /pointcloud；cloud_is_world=true、need_extrinsic=false 仅在 adapter 已正确转换云和传感器原点后使用。原生 lidar 回调不同步 pose/cloud，adapter 的发送顺序不保证处理时配对；I1/I2 测量误差，超限时暂停并评审最小同步改动。

## 执行与停止规则

1. 以实际位姿计算规划系 XY 前馈/误差反馈，再按已确认的命令坐标系旋转。MCU 保留朝向/小陀螺控制；不向旧串口盲送标准 angular.z。
2. 对每条 Bspline 校验有限值、阶次、knots、时间跨度和任务代次。定义跟踪时间与 FSM 的 start_time/frozen 反馈一致；不简单复用原控制器“收到即时间归零”的行为。
3. 到达依据实际位置误差和停稳条件；首期不承诺目标 yaw 控制。取消、失败、输入过期立即禁止旧轨迹重新激活。
4. 轨迹并非必须固定频率更新；不要把“多久没收到新 Bspline”单独当超时。结合有效执行时间、输入年龄、规划器存活与剩余轨迹判断。
5. 保护层检查 NaN、速度/加减速、odom/cloud 新鲜度、TF 与定位跳变、执行授权。硬急停/输入失效停止不能被普通舒适性斜率限制无限推迟；具体停止策略与 MCU watchdog 在 I1 记录。
6. 横移/自转要求全朝向碰撞包络覆盖实际尺寸；不能照搬 Go2 切线朝向双圆柱。
7. 保护进程自身失效不能靠自己发零解决；实车门槛包括验证电控失联停车/独立急停。

## 切换与回退

影子模式只读旧链。实车模式切换时先撤销授权、停车、使旧 controller/behavior 等所有速度源隔离，再授予 SCAN；默认关闭控制授权。不能在两套 launch 同时直发 /cmd_vel 的情况下用一个额外 mux 假装仲裁。

回退时撤销 SCAN 授权并确认停车，停止新执行链，恢复归档的原 launch/参数与唯一旧速度源；不需要还原 LIO/TF/串口源码。当前未更改旧链，不执行切换。


## 下一任务与审查边界

DeepSeek harness 先交付 S0–S3；用户提供 PCD 与尺寸后完成指定地图验证。交付代码差异、可复现启动命令、三种模式用法、测试日志和已知限制。Codex 接到审查请求后检查实现与这些要求的一致性，不把静态审查等同实车安全批准。具体清单见 [交接说明](implementation_handoff.md)。
