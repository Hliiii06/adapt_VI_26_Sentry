# DeepSeek 交接 A：收敛代码与文档，不改变导航行为

日期：2026-10-07。用户决定暂停扩建仿真，转向实车替换准备。
本任务只整理；下一开发任务见 [实车影子接入交接](real_robot_handoff.md)。不要把两项混成一次大重构。

## 目标与边界

让接手者通过一个入口知道“现在有什么、怎么运行、接下来做什么”，无需重读所有历史审查。
保留已有仿真作为回归工具，不再建设 Gazebo、更多地形或更精美显示。
**整理不是删除仿真，更不是宣布实车验收完成。**

实施只在本仓库。三个参考仓库继续只读，不改 RM 的导航、定位、TF、串口。
当前工作树有大量修改和未跟踪成果；先记录 `git status`、diff、未跟踪清单和参考提交。
逐项识别内容来源，不能用 `reset/clean` 恢复成 HEAD；HEAD 不包含全部最新成果。
先形成可追溯的本地基线提交或逐文件备份，敏感文件/生成物不入库；不要盲目 `git add .`。

## 推荐最终阅读入口

尽量复用现有文件，不另造一套同内容的说明：

| 入口 | 唯一职责 |
|---|---|
| 根 README.md | 项目目的、当前阶段、构建、三个常用入口、文档入口 |
| AGENTS.md | 工作边界、参考路径、工程/安全规则；不复制多轮测试报告 |
| docs/README.md | 当前文档导航、准确基线、证据索引；区分当前与历史 |
| architecture/current_navigation.md | RM main 参考架构，注明用户实车验证来源 |
| architecture/scan_planner.md | 明确“上游基线”与“本仓库实施版本”的差异，不能混写 |
| architecture/target_architecture.md | C 目标分层与仍未实现的实车边界 |
| interfaces/* | 当前契约、候选实车契约分别标识，作为接口唯一事实入口 |
| migration/plan.md | 当前里程碑和门槛，不再包含两套完整交接书 |
| migration/progress.md | 简短当前状态、最近变更、下一步；旧逐轮记录移历史 |
| migration/decisions.md | 决策、状态、适用日期、被哪项取代 |
| testing/validation_plan.md | 判据与测试入口，不复制每轮全部结果 |
| testing/pcd_route_fix.md | 当前指定地形支撑面实验操作与限制 |
| testing/mode2_waypoint_z_preview.md | 无支撑面六航点预览操作与限制，与速度闭环严格分开 |
| migration/real_robot_handoff.md | 下一开发任务，仅 I1/I2，不直接驱动车辆 |

`package_inventory.md`、`build_notes.md` 保留作查询资料；`build.md` 是原始需求来源，
保留正文并在顶部注明现阶段入口，不能抹掉用户原始意图。

## 文档整理清单

1. 先写移动/合并清单（旧路径、去向、原因、替代入口），再操作。
2. 建立 `docs/archive/`。以下是归档候选，不是无条件删除名单：
   - migration 下 `baseline_update.md`、`gap_analysis.md`、`s0_baseline.md`、旧 `implementation_handoff.md`；
   - `implementation_report.md` 和 testing 下 `s3_results.md`、`review_round2_fixes.md`：保留实施/审查证据，当前摘要链接到历史全文；
   - `codex_handoff_problems.md`、`field_terrain_check.md`、`tunnel_diagnosis.md`、`height_sweep.md`：含已被推翻的归因，归档页首写清被哪份报告取代；
   - `mode2_field_roundtrip.md` 保留有支撑面对照，可归档但保留可运行命令及航点引用；
   - `terrain_following.md`、`inflation_analysis.md` 先提取仍有效的实现说明再归档历史部分，不能整篇当过期结论处理。
3. `progress.md` 只留当前状态与短摘要；历史按日期搬移，保留失败证据和用户/工具验证区别。
4. 合并重复叙述靠链接，不复制安全边界、话题表、启动命令到五个文件中。
5. 移动 Markdown 后修正**所有入链和相对出链**；旧交接常被外部引用的路径可留短跳转页。
6. `docs/testing/maps/` 不是普通文档：当前 launch、脚本和 YAML 直接引用它。
   本整理批次保持地图、网格、航点、`evidence/` 原路径，不为目录美观搬迁运行资产。
7. 日志在 gitignore 中不代表无价值；不得删除 `log/probes/` 的失败/通过记录。
   历史证据至少保留对应命令、参数快照、地图哈希、结论、限制；本地日志未入库要明说。

### 已确认需要纠正的陈旧表述

- 旧计划/交接“尚未实施”“建议去 SCAN2 工作副本改”：已不适用，实施目录是本仓库 `src/`。
- `interface_mapping.md` 中“所有 adapter 未实现”“没有现成执行端”：区分未实现的 **RM adapter** 与已有的全向 `closed_loop_controller`。
- 上游架构中“双圆柱/先转头/没有 odom 超时”等，只能描述 SCAN2 基线；当前实施有单圆柱配置、全向跟踪和取消/超时改动。
- `progress.md` 底部“地面分割尚未实现”等是历史，不应继续挂在当前下一步。
- 旧“坐标给错”“只能降噪就通行”等归因已被后续证据修正，不能保留为当前事实。
- `inflation_analysis.md` 对 z 方向的解释须以 `grid_map.cpp` 实际膨胀索引复核：障碍按
  `[-z_down,+z_up]` 扩张时，查询点 p 对应原始障碍高度带 `[p.z-z_up,p.z+z_down]`。
  不要把“障碍向上膨胀”误说成“机器人向上包络”；不对称参数尤其容易写反。
- GUI 验证分开记录：三条地形路线由用户确认；六航点最新预览本环境仅无界面测试，不能统称全部 GUI 已验收。
- R=0.26/H=0.25 是用户提供的实施基线；H=0.10、离地预览是诊断条件，不能覆盖真实尺寸。
- 早期“所有场景通过”只属于对应代码与参数快照，不代表当前分支已经全部复跑。

## 代码整理：首批只分类，不大搬家

保留 ROS 包名、消息、目标名、launch 参数、话题、TF、配置默认值和现有启动命令。
不因上游残留 Go2 命名就重命名整个控制/仿真链，不做全仓换行/格式转换。

在现有包清单或新 `scripts/README.md` 中登记这些分组即可：

- 规划核心：`src/planner/{plan_env,path_searching,bspline_opt,plan_manage,traj_utils,scan_planner_msgs}`。
- 潜在实车复用：FSM、GridMap、全向 `closed_loop_controller`、消息与可视化；不是无条件可直接接车。
- 仅仿真：`src/simulator/`、`go2_kinematic_sim`、`open_loop_controller`、`sentry_sim.launch.py`。
- 使用入口：`build.sh`、`run_sentry_sim.sh`、`run_field_route.sh`、`run_waypoint_z_preview.sh`。
- 回归/判据：`scenario*`、`smoke_test.sh`、`check_*`、`test_*`；用途不同不能因名字相似合并。
- 数据准备：`make_*`、`prepare_*`；诊断探针：`probe_navigation.sh`、`dump_cloud.py`、`inspect_field_terrain.py`。

重复代码先列出调用者、差异和建议；没有明确收益就不抽象。若要实际重构，另开小提交与行为回归，
不能把导航逻辑修复藏在文档整理提交里。判据也不能为了“统一”放宽或吞掉非零退出码。

## 交付与验收

- 清理报告包含移动/合并映射、保留的运行入口、剩余未决事项，不再写一篇多轮流水账。
- 文档链接有效；从 README 最多两跳到当前任务、运行说明和接口契约。
- Markdown/注释整理无需重建或再建仿真；脚本路径变更须做语法、导入、launch 解析与相关测试；
  C++ 行为变更不属于此批，另交差异与构建/定向回归。
- 证明参考仓库未改、用户原有修改未丢、地图/证据/失败记录可追溯。
- 完成后停止清理，不继续“优化目录”阻塞下面的 I1/I2 实现。
