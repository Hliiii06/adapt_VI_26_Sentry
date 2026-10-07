# 仓库整理报告（交接 A 交付）

日期：2026-10-07。对应[整理指引 A](cleanup_handoff.md)。任务边界：区分当前说明与历史证据、
合并重复叙述、纠正陈旧表述；**不改导航行为，不做大规模代码搬迁，不删除失败记录**。
下一开发任务见[实车影子接入交接 B](real_robot_handoff.md)，本报告不涉及 I1/I2 实现。

## 一、基线与可追溯性

| 项 | 值 |
|---|---|
| 整理前 HEAD | `5bbc87b` 加大量未提交工作树 |
| 基线提交 | `e35877b`（整理前原样入库，含交接 A/B、`pcd_route_fix`、Mode 2 预览与脚本） |
| 基线快照 | [cleanup_baseline_2026-10-07.txt](../testing/evidence/cleanup_baseline_2026-10-07.txt)（git status、diffstat、参考仓库 SHA） |
| RM `../VI_26_Sentry` | main `7dfe71a`，只读；未跟踪 `src/Sophus/`、`src/hnurm_navigation/BRINGUP_LAUNCH_EXPLAINED.md` 保留 |
| SCAN2 `../../SCAN-Planner-Ros2` | main `103bce4`，干净，只读 |
| SCAN1 `../../SCAN-Planner` | main `f121613` + 用户已有修改，只读 |

整理不 reset、不 clean、不覆盖用户修改；移动一律用 `git mv` 保持内容可追溯。

## 二、移动/合并映射（先写清单，再操作）

去向 `docs/archive/`。归档页首统一加“历史资料”说明：本页被哪份当前文档取代、哪些结论仍有效。

| 旧路径 | 去向 | 原因 | 替代入口（当前） |
|---|---|---|---|
| `docs/migration/baseline_update.md` | `docs/archive/baseline_update.md` | RM `82d0741→7dfe71a` 的变更流水，已完成 | [docs/README 版本基线](../README.md)、[包清单](../architecture/package_inventory.md) |
| `docs/migration/gap_analysis.md` | `docs/archive/gap_analysis.md` | 实施前的 P0–P2 缺口优先级，部分已实施 | [实车交接 B](real_robot_handoff.md) B1/B2 表、[接口映射](interface_mapping.md) |
| `docs/migration/s0_baseline.md` | `docs/archive/s0_baseline.md` | S0 输入快照；用户给定尺寸/外参/限速仍是引用来源 | [实施报告](implementation_report.md) 改动清单、`src/.../config/sentry_*.yaml` |
| `docs/migration/implementation_handoff.md` | `docs/archive/implementation_handoff.md`，原路径留[跳转页](implementation_handoff.md) | 最初的 S0–S3 交接，旧“另选 SCAN 工作副本”权限说明已失效 | [整理指引 A](cleanup_handoff.md)、[实车交接 B](real_robot_handoff.md) |
| `docs/testing/s3_results.md` | `docs/archive/s3_results.md` | 2026-10-01 S3 结果快照（受控碰撞/三模式/失效停止），保留为证据 | [pcd_route_fix](../testing/pcd_route_fix.md)、[验证计划](../testing/validation_plan.md) |
| `docs/testing/review_round2_fixes.md` | `docs/archive/review_round2_fixes.md` | 第二轮复审 4 项修正的当轮证据 | [实施报告](implementation_report.md)、[验证计划](../testing/validation_plan.md) |
| `docs/testing/codex_handoff_problems.md` | `docs/archive/codex_handoff_problems.md` | 原始问题交接，多处归因已被取代 | [pcd_route_fix](../testing/pcd_route_fix.md) |
| `docs/testing/field_terrain_check.md` | `docs/archive/field_terrain_check.md` | 受控洞口判据仍有效；“坐标压在结构上”结论已被取代 | [pcd_route_fix](../testing/pcd_route_fix.md) |
| `docs/testing/tunnel_diagnosis.md` | `docs/archive/tunnel_diagnosis.md` | 已被两轮后续报告更正 | [pcd_route_fix](../testing/pcd_route_fix.md) |
| `docs/testing/height_sweep.md` | `docs/archive/height_sweep.md` | 修复前高度扫描；旧“窄通道天然不过”归因被取代 | [pcd_route_fix](../testing/pcd_route_fix.md)；`robot_height` 旋钮仍有效，见 [README](../../README.md) |
| `docs/testing/inflation_analysis.md` | `docs/archive/inflation_analysis.md` | z 膨胀核对过程；**其 z 方向解释写反、旧计数表作废** | [SCAN 架构：包络参数语义](../architecture/scan_planner.md) |
| `docs/testing/mode2_field_roundtrip.md` | `docs/archive/mode2_field_roundtrip.md` | 有支撑面对照；命令与航点文件保留在归档页 | [Mode 2 无支撑面预览](../testing/mode2_waypoint_z_preview.md) |
| `docs/testing/terrain_following.md`（原文） | `docs/archive/terrain_following_history.md` | 原文含已失效的删点调查叙事 | 当前页仍为 [terrain_following.md](../testing/terrain_following.md)（重写） |
| `docs/migration/progress.md` 历史轮次 | `docs/archive/progress_history.md` | 逐轮流水移历史 | [进度](progress.md)（只留当前状态与短摘要） |

**不搬迁（运行资产，当前 launch/脚本/YAML 直接引用）**：`docs/testing/maps/`（地图、网格、航点）、
`docs/testing/evidence/`（场景原始记录）。**不删除**：`log/probes/` 的失败/通过记录
（在 `.gitignore` 内，不入库，但保留在本地磁盘）。

**保留在当前位置**（当前说明，非历史）：`docs/README.md`、`architecture/*`、`interfaces/*`、
`migration/{cleanup_handoff,real_robot_handoff,plan,progress,decisions,interface_mapping,implementation_report}.md`、
`testing/{validation_plan,pcd_route_fix,mode2_waypoint_z_preview,build_notes,terrain_following}.md`、
根 `README.md`、`AGENTS.md`、`build.md`。

## 三、当前唯一入口职责

| 入口 | 唯一职责 |
|---|---|
| 根 [README.md](../../README.md) | 项目目的、当前阶段、构建、常用入口 |
| [AGENTS.md](../../AGENTS.md) | 工作边界、参考路径、工程/安全规则 |
| [docs/README.md](../README.md) | 文档导航、版本基线、证据索引、当前/历史分界 |
| `architecture/current_navigation.md` | RM main 参考架构（用户实车验证来源） |
| `architecture/scan_planner.md` | 上游基线 + 本仓库实施差异 + 包络参数语义 |
| `architecture/target_architecture.md` | C 目标分层与未实现的实车边界 |
| `interfaces/*` | 当前契约与候选实车契约的唯一事实入口 |
| `migration/plan.md` | 当前里程碑与门槛 |
| `migration/progress.md` | 当前状态、最近变更、下一步 |
| `migration/decisions.md` | 决策、状态、日期、被谁取代 |
| `migration/real_robot_handoff.md` | 下一开发任务 I1/I2（不驱动车辆） |
| `testing/validation_plan.md` | 判据与测试入口 |
| `testing/pcd_route_fix.md` | 当前指定地形支撑面实验与限制 |
| `testing/mode2_waypoint_z_preview.md` | 无支撑面六航点预览（与速度闭环严格分开） |
| `testing/terrain_following.md` | 当前地形分离与高度跟随实现 |
| `docs/archive/README.md` | 历史资料索引与取代关系 |
| [scripts/README.md](../../scripts/README.md) | 脚本分组登记（不搬代码） |
| `migration/cleanup_report.md`（本页） | 本次整理的映射、保留边界、未决事项 |

## 四、执行结果

按上表执行，均用 `git mv`（内容可追溯）：

- **归档**：13 份过程/证据页进入 `docs/archive/`，页首统一加“历史资料”横幅，写明取代它的当前文档与仍有效部分。
- **拆页**：`terrain_following.md` 原文归档为 `terrain_following_history.md`，
  原路径重写为**只含当前实现与操作**的短页（地面/障碍分离、高度跟随链路、RViz 图层、命令、局限）。
- **进展分层**：`progress.md` 只留当前状态、2026-10-06/10-02 摘要、边界与下一步；逐轮流水入 `progress_history.md`。
- **跳转页**：`docs/migration/implementation_handoff.md` 变为 12 行短页，指向归档与当前任务。
- **新增**：[docs/archive/README.md](../archive/README.md)（归档索引）、[scripts/README.md](../../scripts/README.md)（脚本分组登记）、本报告。
- **未搬迁**：`docs/testing/maps/`、`docs/testing/evidence/` 全部原路径；**未删除**任何失败记录或本地 `log/probes/`。
- **未改代码**：`src/` 零改动（`git diff --name-only HEAD -- src` 为空）；脚本只新增说明文件。

## 五、陈旧表述纠正清单

| 位置 | 旧表述 | 纠正 |
|---|---|---|
| `interface_mapping.md` | “所有 adapter 均为 PROPOSED，本轮未实现”“Bspline execution：无现成执行端 / MISSING” | 区分**未实现的 RM 输入/命令 adapter** 与**已实施的 `closed_loop_controller` 全向执行端**；逐行加“本仓库实施状态”列 |
| `scan_planner.md` | 全文默认按上游 `103bce4` 描述，读者易误当当前实现 | 新增“上游基线与本仓库实施版本”差异表；关键处标“**本仓库实施**” |
| `scan_planner.md`/`implementation_report.md`/`inflation_analysis.md` | 把 `obstacles_inflation_z_up/down` 说成“机器人包络向哪边扩”，并写成 `z ± 膨胀` | 按 `grid_map.cpp::rebuildInflationOffsets/updateInflationLayer` 复核：参数是**障碍膨胀方向**，查询点 `p` 对应障碍高度带 `[p.z − z_up, p.z + z_down]`；仅对称时可写 `z ± 膨胀` |
| `inflation_analysis.md`（已归档） | “减小 down / 减小 up 谁有效”的计数表与结论 | 标注**计数表作废**（用反了的约定算出，属 INFERRED 反推）；保留“上游确为 up=.1/down=.4”“对称 ±.125 覆盖 `[0,0.25]`”“压缩包络到小于机体尺寸不安全”等仍有效结论 |
| `progress.md` 底部 | “地面分割尚未实现”“下一步交 Codex 复审”等挂在当前 | 历史移入 `progress_history.md`；当前页写清已实施与下一步是交接 B |
| `plan.md` | “本轮只更新文档”“先交付 S0–S3”等当任务 | 改为当前顺序（A 已完成、B 待批准），删去与交接 B 重复的 I1–I3 接口草案 |
| `s3_results.md`/`review_round2_fixes.md`（已归档） | 易被当作当前证据 | 横幅注明“当轮快照”，指向当前判据与路线修复页 |
| `field_terrain_check.md`（已归档） | “是坐标点压在结构上”“可行驶起伏只有 0.14 m” | 横幅注明已被 `pcd_route_fix.md` 取代；保留仍有效的 0.25 m 阈值判据 |
| `target_architecture.md` | “本轮完成规划，未授权执行导航代码修改” | 改为“S0–S3 已完成，实车接入未开始” |
| `validation_plan.md`、`build_notes.md` | “不是本轮已执行测试”“本轮不执行构建” | 注明仿真阶段已执行、当轮证据在归档；`build_notes` 明确 RM 全量构建仍未验证 |
| 根 `README.md`/`AGENTS.md` | 指向已归档路径 | 链接改到 `docs/archive/*`；补归档使用约定 |
| `build.md` | 顶部无提示，内部目录树列旧文档名 | 顶部加“当前入口提示”，正文原文保留 |

GUI 验证口径统一为：**三条地形路线由用户确认；六航点最新预览本环境仅无界面测试**，
不能统称 GUI 已验收。真实尺寸 R=0.26/H=0.25 与诊断 H=0.10、离地预览始终分开标注。

## 六、验证

| 项 | 方法 | 结果 |
|---|---|---|
| 本地链接 | 脚本遍历 `docs/**/*.md` 与根 markdown 的 `[..](..)` 本地目标 | **41 份文档 / 346 个本地链接 / 0 断链**（含审查修正后复检） |
| 代码围栏 | 检查每文件 ``` 计数为偶 | 0 个不平衡文件 |
| 陈旧路径 | grep 旧路径（`testing/s3_results.md` 等） | 除归档页与 `build.md` 原始正文（已加横幅）外无残留 |
| 代码未改 | `git diff --name-only HEAD -- src` | 空 |
| 地图/证据 | `git status --porcelain -- docs/testing/maps docs/testing/evidence` | 仅新增基线快照，其余原路径未动 |
| 参考仓库 | `git -C ... status/rev-parse` | RM `7dfe71a` 只读（2 项未跟踪保留）；SCAN2 `103bce4` 干净；SCAN1 `f121613` + 6 改 2 未跟踪，均未动 |
| 用户修改 | 基线提交 `e35877b` 与整理前快照 | 全部保留，未用 `reset/clean`，未覆盖 |
| 构建/运行 | 本批次是否执行 | **未构建、未启动 ROS、未跑场景**；文档整理无需重建 |

## 七、保留的运行入口（未改名）

`scripts/build.sh`、`scripts/run_sentry_sim.sh`、`scripts/run_field_route.sh`、
`scripts/run_waypoint_z_preview.sh`、`scripts/scenario.sh`、`scripts/smoke_test.sh`、
`scripts/probe_navigation.sh` 及全部 `check_*/make_*/prepare_*/test_*`。
用途与不合并理由见 [scripts/README.md](../../scripts/README.md)。
`docs/testing/maps/` 的 YAML/网格/PCD 与 `docs/testing/evidence/` 的场景记录路径不变。

## 八、未决与本次未做

- **交接 B（I1/I2）未开始**：等待用户批准；本报告不含任何实车/影子实现结论。
- 未重构脚本或 C++；若后续抽象公共逻辑，另开小提交并做行为回归。
- 未重跑历史场景矩阵；旧“全部场景通过”只属于对应快照。
- `inflation_analysis.md` 的正确约定计数未重算（原表作废）；如需要，应按
  `[p.z − z_up, p.z + z_down]` 重写一个可复现的核对脚本，再给出数值。
- 未推送远端（本地领先，推送前仍需确认无敏感文件）。

## 九、审查修正（对照 `37b22ac`）

审查指出 2 处文档错误（P2）与 2 处小问题，均已按代码复核修正；本轮同样**只改文档**：

| 问题 | 复核依据 | 修正 |
|---|---|---|
| Mode 1 高度语义被写成无条件地形跟随 | `scan_replan_fsm.cpp`：`ground_grid_file` 非空时 `terrain_following_=true`（56–60 行）；仅该分支用 `ground_map_.heightAt(...) + body_height_`（186–195 行），否则用首个 odom 记录的 `rviz_goal_height_`（436–438 行） | `plan.md` 列出有/无网格两种行为；`scan_planner.md` 差异表与能力边界、`implementation_report.md` 改动清单同步加条件；`terrain_following.md` 链路表加“无网格时”分支 |
| “当前地形实现”引用旧算法 | `prepare_terrain_map.py`：`--ground-mode` 默认 `local`（低分位数 + 补洞 + 中值平滑，256–269 行），`global` 才是旧二次曲面拟合 | `terrain_following.md` 与 `scripts/README.md` 按实际默认改写；RMS 等旧数值只留归档 |
| “地面点绝不能进占据栅格”过宽 | 无支撑面预览刻意保留原始地面点 | 限定为**贴地支撑面实验**；`terrain_following.md`、`validation_plan.md` 同时点明与无支撑面预览的区别 |
| 报告扩展名混淆 | `prepare_terrain_map.py` 写 `_report.txt`（332 行）；`prepare_route_terrain.py` 写 `_report.json`（108 行） | `terrain_following.md`、`scripts/README.md` 按工具分别写明 |

修正后复检：41 份文档 / 346 个本地链接 / 0 断链；代码围栏平衡；`src/` 与运行资产仍零改动。
