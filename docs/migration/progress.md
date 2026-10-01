# 调研与规划进展

日期：2026-10-01。当前阶段：**S0–S3 已实施并跑通仿真，等待 Codex 审查；实车接入未开始。**

## S0–S3 实施结果（本轮）

- **S0**：记录用户给定的 PCD、半径 0.26 m、高 0.25 m、雷达外参、MPPI 限速；解析 PCD
  发现地面高度散布 -0.06..0.22 m。见 [S0 基线](s0_baseline.md)。
- **S0**：把 SCAN-Planner-Ros2 main `103bce4` 的 9 个包纳入本仓库 `src/`，未修改参考仓库。
- **S1**：取消「先对齐轨迹 yaw 才平移」的门槛；新增 `yaw_mode`(hold/align/spin)；
  碰撞包络改为 offset=0 的单圆柱（半径 0.26 + 余量，高度带 [0, 0.25]），
  由于所有查询都走 `getInflateOccupancy`，碰撞代价/安全检查/RViz 显示语义自动一致；
  新增 odom 陈旧、非法轨迹、`planning/reset` 取消等保护；收轨迹时按 `start_time` 对齐执行时间。
- **S2**：`map_pub` 增加地面过滤与 z 归零；`pcl_render_node` 增加可配置雷达外参并使射线原点、
  `sensor_pose`、`world→sensor` TF 使用同一外参；新增 `sentry_sim.launch.py`（`/sentry_sim`
  命名空间）与 RViz 配置，三模式全部保留。闭环为真实速度闭环（速度积分反馈 odom），
  未使用 open_loop_controller。
- **S3**：9 个场景验证全部通过，详见 [S3 结果](../testing/s3_results.md)：
  车头不变横移（dx=0.000、朝向偏离 0.00°）、取消门槛（旋转 90° 同时平移到位）、
  边转边走（自转 179.9° 仍走直线）、三模式全部到达、贴障通过（独立净空 0.345 m ≥ 0.26）、
  阻挡目标不动、取消后 0.670 s 停车、里程计断流后 2599 个采样全为 0。
- **明确未验证**：RViz 图形交互（本环境无法创建 OpenGL 上下文）、实车、爬坡、加速度上限合理性。
- 环境适配：本机 MVS SDK 的 libusb 会破坏 PCL 运行期符号解析，`run_sentry_sim.sh` 中剔除；
  这是环境修复，不是源码改动。

本轮修改的代码在 `src/`，清单见 [实施报告](implementation_report.md)。RM 与两个 SCAN
参考仓库均未修改。

## 前一轮已完成

- 按用户要求将顺序调整为 S0–S3 独立 SCAN 全向适配/PCD/RViz 闭环仿真 → Codex 审查 → I1–I4 RM 接入。
- 明确保留 Mode 1/2/3，允许必要的碰撞、odom 朝向、跟踪、FSM 时序与仿真局部代码修改。（该轮只编辑文档；实施已在其后完成，见上文。）
- 增加 harness 交接说明与通俗仿真验收表。
- PCD 路径、实际外形尺寸与雷达外参已由用户提供，见 [S0 基线](s0_baseline.md)。

## 前一轮已完成

- 复核用户拉取后的 RM main 7dfe71a（旧基线 82d0741），并同步架构、包清单、输入/TF/服务和构建说明。
- 记录点云坐标/时间、高频速度源、已删除视觉包及新消息草稿等变化，详见 [基线更新](baseline_update.md)。
- 按用户说明纠正目标：采用 SCAN 当前空间避障/参考高度实现，不再要求先做完整地形全局规划。
- 明确现有 RM 的 RViz 2D goal 实车导航由用户验证；主机小陀螺代码不代表 MCU 实际行为。
- C 标记为已认可，制定 P0–P5 实施计划、候选包/接口、单一输出、停止/回退及验证门槛。

## 保留的前期成果

两个项目架构、包/节点、launch、Nav2 职责、SCAN 地图/优化/执行、话题/QoS/frame/time、TF、actions 和串口字段均保留并按新基线修订。ROS1 注释仍只作为理解参考。

## 验证范围

以下描述的是**更早的纯文档轮**，不是当前状态；当前 S0–S3 的构建与仿真证据见上文与
[S3 结果](../testing/s3_results.md)。

早期文档轮仅做静态源码/差异与文档检查，不编译、不启动节点、不回放、不做实车测试。历史中止构建记录保留在 [构建说明](../testing/build_notes.md)，不能当成新 main 的构建结果。

RM/SCAN2 在该轮无受跟踪源码修改，RM 两项未跟踪用户文件保留；SCAN1 原有改动不恢复、不覆盖。当时迁移目录还不是 Git 仓库（现已仓库化，见下）。

前一轮交付检查通过：18 份文档、73 个本地链接均可解析，代码围栏闭合；旧基线仅保留在历史对比处。RM git diff --check 通过，RM/SCAN2 无受跟踪修改，SCAN1 原有状态保留。新方案参数和包名未写成已实现接口。

## 工作区仓库化（2026-10-01）

按用户要求把本迁移目录变成私有仓库：`git@github.com:Hliiii06/adapt_VI_26_Sentry.git`，主分支 `main`。入库范围是文档、`src/`（SCAN 实施副本）、`scripts/`、`README.md`、`AGENTS.md`、`build.md` 与 `.gitignore`；`artifacts/`、`log/`、`build/`、`install/` 按用户决定**不入库**，仍保留在本地磁盘供追溯。参考仓库 `../VI_26_Sentry`、`../../SCAN-Planner-Ros2`、`../../SCAN-Planner` 未被初始化、提交或推送，各自状态与基线不变。

这是工作区管理动作，不是文档轮结论：不改变上面任何架构判断、阶段状态或 PROPOSED 标记。

## 下一步

交给 Codex 审查 S0–S3：改动清单见 [实施报告](implementation_report.md)，测试证据见
[S3 结果](../testing/s3_results.md) 与 [evidence/](../testing/evidence/)。重点建议按
[交接说明](implementation_handoff.md) 结尾的审查清单：三模式回归、odom/yaw 与速度转换、
碰撞检查各入口一致性、未来姿态假设、是否真正速度闭环、时间/取消/失效停止、
模拟与实车隔离、构建与操作说明可复现性。

已知需要审查者特别关注的点：

1. `collision_block` 的拒绝来自动力学可行性检查而非碰撞代价单独否决——安全但证据不完整。
2. 启动瞬间 FSM 会重试数次 `The robot is inside an obstacle`（疑似越界返回 -1 被当作占据）。
3. 地面过滤阈值 0.30 m 会一并剔除矮障碍，是 PCD 质量的直接后果。
4. RViz 图形交互未验证，需要用户在带显示的机器上确认 2D Goal Pose 可用。

审查通过后再开展 I1–I4 实车接入；当前未改 RM 源码、未接实车。
