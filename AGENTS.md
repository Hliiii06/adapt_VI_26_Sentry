# 迁移工作区指南

这是 ROS 2 全向哨兵机器人从 Nav2 向 SCAN-Planner 及其他算法迁移的工作区。2026-10-07 用户决定**暂停扩建仿真，进入仓库整理与 I1/I2 实车数据影子接入准备**；尚未授权底盘输出或实车运行。

**交接 A 仓库整理已完成**（[整理报告](docs/migration/cleanup_report.md)）；[交接 B](docs/migration/real_robot_handoff.md)
的 **B1/B2 已实现**：`/sentry_scan` 影子入口只读真实输入、无底盘输出，契约见
[影子输入契约](docs/interfaces/shadow_input_contract.md)，结果见[影子验收证据](docs/testing/shadow_acceptance.md)。

**当前路径（2026-10-09 起）**：没有 ROS bag，改为**实车在线只读影子验证**——
流程见[在线影子运行手册](docs/migration/onsite_shadow_runbook.md)，
接管方案为 **PROPOSED**（[接管方案](docs/migration/real_robot_takeover_plan.md)），
**未授权发布真实底盘命令**；受控驱车须 Codex 审查 + 用户明确同意。
在线影子阶段**不向实车注入故障**（故障注入只针对影子链路），首轮只做平坦空旷区域。
实施仍在本仓库，RM 和 SCAN 参考仓库继续只读。
保留已有仿真作回归，不扩展 Gazebo，不把无支撑面的开环航点预览接到实车。
历史过程与证据在 [docs/archive/](docs/archive/README.md)：引用归档页必须写明“历史记录”及取代它的当前文档，
不能把其中的旧归因当作当前事实；不删除失败记录，不搬迁 `docs/testing/maps/`、`docs/testing/evidence/`。

## 范围与源码

- 用户已指定：文档维护在本目录；RM 参考 `../VI_26_Sentry` 的 **main**。
- ROS 2 SCAN：`../../SCAN-Planner-Ros2`；ROS 1 注释参考：`../../SCAN-Planner`。
- 精确提交、脏文件及证据入口见 [docs/README.md](docs/README.md)。不要把 `feature/odin` 或已有 `install/` 的行为当成 main 源码事实。
- 本仓库 `git@github.com:Hliiii06/adapt_VI_26_Sentry.git`（主分支 `main`）自 **2026-10-09 起是公开仓库**（用户确认）。公开意味着**当前内容与全部历史**对所有人可见：每次推送前必须扫描**整个历史**（`git grep <模式> $(git rev-list --all)`，不只是本次 diff），确认没有密钥、口令、token、串口/设备配置、内网地址等敏感信息。入库内容是文档、`scripts/`、`AGENTS.md`、`README.md`、`build.md` 与 `src/`（纳入本仓库的 SCAN 源码）；`artifacts/`、`log/`、`build/`、`install/` 由 `.gitignore` 排除，编译产物不入库，也不要用 `git add -f` 绕过。
- 参考仓库（`../VI_26_Sentry`、`../../SCAN-Planner-Ros2`、`../../SCAN-Planner`）各有自己的远端，不要在本仓库提交或推送它们的源码与改动；本仓库正常提交/推送无需再逐次征求许可，但**推送前**必须完成上述敏感信息扫描（公开仓库尤其如此），并在回复里说明扫描范围与结论。
- **`src/` 是实施副本，不是参考仓库**：来源为 SCAN-Planner-Ros2 main `103bce4`，改动边界见 [实施报告](docs/migration/implementation_report.md)。不要再把 `../../SCAN-Planner-Ros2` 当作待改代码；它保持只读。RM 的 `../VI_26_Sentry` 始终只读。
- 当前阶段：S0–S3 与地形/洞口工作已完成；**I1/I2 影子接入已实现并通过合成输入契约测试**，
  尚未接入 VI_26_Sentry 实车、未授权底盘输出。RViz 图形交互在本环境无法验证。
  实施入口见 [实施报告](docs/migration/implementation_report.md)，影子实现见
  [影子输入契约](docs/interfaces/shadow_input_contract.md)，地形见 [地形与高度跟随](docs/testing/terrain_following.md)，
  指定路线与洞口见 [真实 PCD 路线修复](docs/testing/pcd_route_fix.md)。

用户最新确认：三维导航指 SCAN 当前的三维占据/空间避障与参考高度实现，不要求自由三维或完整地形通行规划；现有 RM 配置已由用户验证 RViz 2D goal 规划并驱动实车；小陀螺部分由电控负责，仓库相关源码不代表实际执行路径。C 方向已认可，实施结果见 [实施报告](docs/migration/implementation_report.md)。

2026-10-02 最新：用户批准先打通指定洞口与坡道的有界支撑层选择；已修复 A* 下坡取整与模拟雷达虚增洞顶厚度。
真实大坡 H=0.25 m、两洞 H=0.10 m 已闭环通过；两洞 H=0.25 m 仍拒绝，不得宣称实车可通过。
后续接手优先读 [真实 PCD 修复与入口](docs/testing/pcd_route_fix.md)，其中结果取代旧的最低层/窄通道归因。
不扩大 Gazebo 或全场多层规划，不改 RM；默认真实尺寸与诊断缩小尺寸必须明确区分。

当前实施顺序与交付边界以[实施计划](docs/migration/plan.md)和[实车交接 B](docs/migration/real_robot_handoff.md)为准；最初的 harness 交接已归档，原路径 [implementation_handoff.md](docs/migration/implementation_handoff.md) 只是跳转页。

## 开始工作前

修改导航架构前必须阅读：

1. [当前架构](docs/architecture/current_navigation.md)、[SCAN 架构](docs/architecture/scan_planner.md)。
2. [候选目标架构](docs/architecture/target_architecture.md)。
3. [ROS topics](docs/interfaces/ros_topics.md)、[服务与 actions](docs/interfaces/ros_services_actions.md)、[TF](docs/interfaces/tf_tree.md)、[底盘接口](docs/interfaces/chassis_interface.md)。
4. [迁移计划](docs/migration/plan.md)、[接口映射](docs/migration/interface_mapping.md)、[决策](docs/migration/decisions.md)、[进展](docs/migration/progress.md)。

目录、包和依赖见 [包清单](docs/architecture/package_inventory.md)。以后每次工作先核对实际源码、分支、`git status`，再读取对应源码目录下的 `AGENTS.md`。

## 工程规则

- Prefer minimal changes. Do not perform broad refactoring unless explicitly required.
- 接口连接 prefer adapters；全向适配允许必要的局部 SCAN 碰撞、odom 姿态、跟踪、FSM 时序及仿真代码修改。不要以“保留核心”为由遗漏碰撞代价/安全检查，也不做无关重构。
- 默认保留 topic 名称、message 类型、TF 含义、frame 名和底盘协议。架构调整先形成可评审建议，经用户同意再实施。
- 实施范围已限定为 SCAN 相关改动与仿真/测试（在 `src/` 内）；不得顺带修改 RM 的 LIO、配准、TF、Nav2 配置、串口或固件。改动前后都保留用户已有修改，禁止覆盖用户工作。
- 结论区分 `CONFIRMED`、`INFERRED`、`UNKNOWN`；候选设计标 `PROPOSED`。每个关键结论给出文件及函数/参数。未由本轮运行验证的内容不能声称本轮实测；用户已验证的 RM 导航须明确标注证据来自用户，不推广为 SCAN 适配已验证。
- 保留用户未提交文件，不使用 `reset --hard`、`clean -fd`。不要自动恢复不是自己产生的修改。
- 改接口需记录：topic、type、publisher、subscriber、QoS、frame、时间戳来源及频率。
- 改 TF 需记录：parent、child、publisher、pose source、频率和原因；禁止 silent TF change。
- 测试脚本必须只终止**本次启动**的进程：仿真用 `setsid` 放进独立进程组，清理只 `kill -- -PGID`；不要用 `pkill -f scan_planner_node` 这类宽泛匹配，它会误杀同一用户的其它实验。判据脚本失败必须返回非零，不要用"第一次速度到零"这类近乎恒真的条件。
- 新依赖先记录名称、用途、代码位置、当前可用性，再决定安装；不要随意 `apt install`、`pip install`、`git clone`。

## 验证与安全

涉及 cmd_vel、轨迹、TF、定位、里程计、碰撞、速度/加速度或底盘控制时，`colcon build` 成功不等于完成。按静态验证 → 构建 → 仿真/rosbag → 可视化 → 受控实车测试推进，见 [验证计划](docs/testing/validation_plan.md)。仿真阶段已执行（当轮结果见[归档 S3 结果](docs/archive/s3_results.md)、[真实 PCD 路线修复](docs/testing/pcd_route_fix.md)）；**RViz 图形交互在本环境无法验证**（无法创建 OpenGL 上下文），实车测试未开始。

SCAN 的 `open_loop_controller` 会直接发布模拟里程计，不是实车速度接口。不能因名字有 controller 就接入定位话题。两个规划器并存时，最终速度输出必须有单一授权来源。

## 常用只读命令

```bash
# 本仓库（文档 + 实施）
git status --short --branch
git log --oneline -5
scripts/build.sh                       # 隔离构建（产物在 build/install，日志在 log/ros）
scripts/run_sentry_sim.sh navi_mode:=1 # 启动闭环仿真
scripts/scenario.sh mode1_lateral      # 场景验证，结果在 log/scenarios/
scripts/scenario.sh terrain_ramp20     # 合成 20° 上坡
scripts/scenario.sh terrain_down       # 同一坡反向（下坡）
scripts/scenario.sh terrain_crest      # 上坡->平台->下坡（跨越坡顶）
scripts/scenario.sh terrain_tunnel_high # 可控洞口：高洞应通过
scripts/scenario.sh terrain_tunnel_low  # 可控洞口：低洞应拒绝
scripts/scenario.sh terrain_lateral    # 横向坡面绕障：规划高度 vs 执行高度
scripts/scenario.sh cancel_race        # 规划期间取消 + 注入延迟旧授权/新时间戳轨迹
scripts/scenario.sh goal_out_of_grid   # 目标在已知地面之外：拒绝规划且不动
# 降低机器人高度做可通行性排查：robot_height 是唯一高度旋钮（派生 body_height
# 与 z 包络）。历史高度扫描见 docs/archive/height_sweep.md
scripts/run_sentry_sim.sh navi_mode:=1 robot_height:=0.10 ...
scripts/prepare_terrain_map.py --input ~/pcd_map/rmuc2026_field.pcd \
  --out-prefix docs/testing/maps/field/rmuc2026   # 真实场地地形分离

# 参考仓库（只读）。注意 SCAN-Planner / SCAN-Planner-Ros2 在 /home/hzq 下，
# 不在本仓库父目录 nav/ 下——核查参数时别搞错路径（曾因此误判一次）。
grep -n obstacles_inflation /home/hzq/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml
git -C ../VI_26_Sentry status --short --branch
git -C ../VI_26_Sentry rev-parse HEAD
git -C ../../SCAN-Planner-Ros2 status --short --branch
rg --files ../VI_26_Sentry/src -g package.xml -g '*launch*.py'
rg -n 'create_subscription|create_publisher|sendTransform' ../VI_26_Sentry/src/hnurm_bringup
git -C ../VI_26_Sentry diff --check
```

后续编译基于 ROS 2 Humble + colcon；RM 含嵌套工作区和依赖声明差异，不能把全量编译视为已确认标准流程。SCAN README 给出 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`。隔离目录、依赖检查和本轮未构建状态见 [构建说明](docs/testing/build_notes.md)。

完成任务时更新 [progress.md](docs/migration/progress.md)；确认参考仓库无源码修改，文档链接有效，未把 proposed 决策写成已批准。
脚本分组见 [scripts/README.md](scripts/README.md)；整理时先写移动/合并映射再操作（本次模板见 [cleanup_report.md](docs/migration/cleanup_report.md)）。
