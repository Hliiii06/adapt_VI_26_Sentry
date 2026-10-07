> **历史资料（2026-10-07 归档）**：本文是**实施前**的适配缺口与 P0–P2 优先级判断，
> 其中大部分已在 `src/` 内实施。当前缺口、输入契约与影子运行要求见
> [实车影子接入交接](../migration/real_robot_handoff.md)；逐项映射见[接口映射](../migration/interface_mapping.md)。
> “已澄清，不再列为阻断”一节作为历史判断保留。> 归档映射与保留边界见[整理报告](../migration/cleanup_report.md)。

# C 方案适配缺口

以用户已验证的 RM 导航为基线；以下是新增 SCAN 消费者的适配工作，不是对现有系统故障的断言。证据 R/S 编号见 [入口](../README.md)。表中 P0/P1/P2 是风险优先级，不是最新阶段编号：先完成 S0–S3 独立全向仿真与审查，再做 I1–I4 接车。RM 速度来源、TF 交接、串口与固件问题不阻塞独立模拟环境开发；姿态/碰撞/轨迹时钟/模拟失效停车则须首期覆盖。

| 优先级 | 项目 | 事实与计划 |
|---|---|---|
| P0 | 输入坐标/参考点 | SCAN callbacks 不查 TF，Bspline 无 frame；R6/R7 存在不同 frame/原点解释；统一到显式规划系，不仅改 header |
| P0 | 有效速度 | 主 /Odometry 未填 twist；新默认开启 /LIVO2/imu_propagate，已填 vel_end；核对 world 数值系、IMU 原点、stamp，优先评估该源 |
| P0 | 全向执行与电控 | S8 强制 yaw 对齐；用户确认 MCU 承担部分旋转。新增不依赖对齐的 XY follower，核对真正命令轴，不改小陀螺固件 |
| P0 | 碰撞包络 | S5/S7 使用切线朝向双圆柱；实际横移/旋转需保守全朝向覆盖，优先现有参数表达 |
| P0 | 唯一输出及失效停止 | controller/behavior 等都可能输出；影子模式隔离，实车前验证授权切换、输入失效、进程崩溃及 MCU watchdog |
| P1 | 时序与传感器 pose | 高频 odom 用 IMU stamp，主云/odom 用 now；filter 已改为云 stamp TF，但 SCAN lidar 仍取缓存 pose；度量配对误差，超限拒绝 |
| P1 | 点云输入 | /pointcloud 现为 base_footprint 且经过地面分割和高度截断；优先评估完整 /cloud_registered 与正确射线原点 |
| P1 | 任务生命周期 | SCAN topics 缺少任务结果/取消；新增 facade，明确 RViz topic 与 Nav2 action 的差异；竞赛 BT 后迁移 |
| P1 | 轨迹时钟 | 新 follower 必须与 FSM start_time/frozen 契约一致；不能只去掉原控制器的 yaw 门槛而忽略时钟 |
| P1 | 路线能力 | Mode 1 不是地图全局搜索，Mode 3 依赖参考高度；限制场景、明确不可达失败，不把复杂地形全局规划作为当前前置 |
| P1 | 重定位与 TF | 保留已验证旧链；新增执行层检测跳变、使旧轨迹失效；R8 状态不能单独证明定位有效 |
| P1 | 未知/越界 | S4/S5 查询语义需专项验证，显示空白不等于可安全通行 |
| P2 | 依赖与旧入口 | 视觉包被删除但 manifest/外部脚本可能残留；按最小依赖构建，不默认恢复全部旧包 |
| P2 | 草稿接口 | new_msg 未纳入消息生成，串口/视觉重写不与 SCAN MVP 捆绑 |
| P2 | 可视化 | 部分 marker 硬编码 world/map；需要显示适配，不广播虚假单位 TF 掩盖错位 |

## 已澄清，不再列为阻断

- RM 当前实车导航是否可用：用户已确认 RViz 2D goal 规划和行驶成功。
- 是否要求真正三维/完整地形导航：不要求，以 SCAN 当前实现为目标。
- 是否采用 C：用户已认可。
- row/roll 拼写与旧 description 重复链：配置已修正，description 已删除。
- filter 最新 TF、区域订阅：当前按消息时刻查询，区域端点已删除。

机身尺寸与仿真阈值在 S0 记录；实车启动/overlay、实际电控轴与单位、时间延迟和停车阈值在 I1 接车前确认。不能从仓库小陀螺分支推断固件运行情况。
