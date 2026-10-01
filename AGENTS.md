# 迁移工作区指南

这是 ROS 2 全向哨兵机器人从 Nav2 向 SCAN-Planner 及其他算法迁移的调研工作区。当前阶段为 **独立 SCAN 全向适配与仿真交接规划**；先仿真并审查，再接 RM。

## 范围与源码

- 用户已指定：文档维护在本目录；RM 参考 `../VI_26_Sentry` 的 **main**。
- ROS 2 SCAN：`../../SCAN-Planner-Ros2`；ROS 1 注释参考：`../../SCAN-Planner`。
- 精确提交、脏文件及证据入口见 [docs/README.md](docs/README.md)。不要把 `feature/odin` 或已有 `install/` 的行为当成 main 源码事实。
- 本目录自 2026-10-01 起是**私有 Git 仓库**：`git@github.com:Hliiii06/adapt_VI_26_Sentry.git`，主分支 `main`。入库内容是文档、`scripts/`、`AGENTS.md`、`README.md`、`build.md` 与 `src/`（纳入本仓库的 SCAN 源码）；`artifacts/`、`log/`、`build/`、`install/` 由 `.gitignore` 排除，编译产物不入库，也不要用 `git add -f` 绕过。
- 参考仓库（`../VI_26_Sentry`、`../../SCAN-Planner-Ros2`、`../../SCAN-Planner`）各有自己的远端，不要在本仓库提交或推送它们的源码与改动；本仓库正常提交/推送无需再逐次征求许可，但**推送前**确认没有密钥、串口配置等敏感文件被新增进跟踪范围。
- **`src/` 是实施副本，不是参考仓库**：来源为 SCAN-Planner-Ros2 main `103bce4`，改动边界见 [实施报告](docs/migration/implementation_report.md)。不要再把 `../../SCAN-Planner-Ros2` 当作待改代码；它保持只读。RM 的 `../VI_26_Sentry` 始终只读。
- 当前阶段：**S0–S3 已实施完成（SCAN 全向适配 + PCD/RViz 闭环仿真已跑通），等待 Codex 审查**，尚未接入 VI_26_Sentry 实车。历史文档轮的限制（只改文档、不构建运行）已被用户后续的实施指示取代。实施入口见 [实施报告](docs/migration/implementation_report.md)。

用户最新确认：三维导航指 SCAN 当前的三维占据/空间避障与参考高度实现，不要求自由三维或完整地形通行规划；现有 RM 配置已由用户验证 RViz 2D goal 规划并驱动实车；小陀螺部分由电控负责，仓库相关源码不代表实际执行路径。C 方向已认可，实施结果见 [实施报告](docs/migration/implementation_report.md)。

最新实施顺序与交付边界以 [实施计划](docs/migration/plan.md)和 [harness 交接说明](docs/migration/implementation_handoff.md)为准：保留三种模式，适配 odom 朝向、全向跟踪和碰撞检查；用用户 PCD 完成 RViz 闭环运动仿真，再审查、再接入 VI_26_Sentry。

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
- 新依赖先记录名称、用途、代码位置、当前可用性，再决定安装；不要随意 `apt install`、`pip install`、`git clone`。

## 验证与安全

涉及 cmd_vel、轨迹、TF、定位、里程计、碰撞、速度/加速度或底盘控制时，`colcon build` 成功不等于完成。按静态验证 → 构建 → 仿真/rosbag → 可视化 → 受控实车测试推进，见 [验证计划](docs/testing/validation_plan.md)。仿真阶段已执行（结果见 [S3 结果](docs/testing/s3_results.md)）；**RViz 图形交互在本环境无法验证**（无法创建 OpenGL 上下文），实车测试未开始。

SCAN 的 `open_loop_controller` 会直接发布模拟里程计，不是实车速度接口。不能因名字有 controller 就接入定位话题。两个规划器并存时，最终速度输出必须有单一授权来源。

## 常用只读命令

```bash
# 本仓库（文档 + 实施）
git status --short --branch
git log --oneline -5
scripts/build.sh                       # 隔离构建（产物在 build/install，日志在 log/ros）
scripts/run_sentry_sim.sh navi_mode:=1 # 启动闭环仿真
scripts/scenario.sh mode1_lateral      # 场景验证，结果在 log/scenarios/

# 参考仓库（只读）
git -C ../VI_26_Sentry status --short --branch
git -C ../VI_26_Sentry rev-parse HEAD
git -C ../../SCAN-Planner-Ros2 status --short --branch
rg --files ../VI_26_Sentry/src -g package.xml -g '*launch*.py'
rg -n 'create_subscription|create_publisher|sendTransform' ../VI_26_Sentry/src/hnurm_bringup
git -C ../VI_26_Sentry diff --check
```

后续编译基于 ROS 2 Humble + colcon；RM 含嵌套工作区和依赖声明差异，不能把全量编译视为已确认标准流程。SCAN README 给出 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`。隔离目录、依赖检查和本轮未构建状态见 [构建说明](docs/testing/build_notes.md)。

完成任务时更新 `progress.md`；确认参考仓库无源码修改，文档链接有效，未把 proposed 决策写成已批准。
