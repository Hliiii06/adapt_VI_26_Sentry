# 历史归档索引

本目录存放**已完成或已被取代**的过程记录与证据。它们不是当前入口，但内容原样保留：
失败记录、错误判断、参数快照与当轮实测都不得因归档而删除或改写。

- 当前入口：[docs/README.md](../README.md)
- 本次整理的映射与保留边界：[整理报告](../migration/cleanup_report.md)

## 归档内容与被谁取代

| 归档页 | 性质 | 当前替代入口 |
|---|---|---|
| [baseline_update.md](baseline_update.md) | RM `82d0741→7dfe71a` 变更流水 | [版本基线](../README.md)、[包清单](../architecture/package_inventory.md) |
| [gap_analysis.md](gap_analysis.md) | 实施前的 P0–P2 适配缺口 | [实车交接 B](../migration/real_robot_handoff.md)、[接口映射](../migration/interface_mapping.md) |
| [s0_baseline.md](s0_baseline.md) | S0 输入快照（用户给定尺寸/外参/限速，仍为引用来源） | [实施报告](../migration/implementation_report.md) |
| [implementation_handoff.md](implementation_handoff.md) | 最初的 S0–S3 harness 交接 | [整理指引 A](../migration/cleanup_handoff.md)、[实车交接 B](../migration/real_robot_handoff.md) |
| [s3_results.md](s3_results.md) | 2026-10-01 S3 结果快照（受控碰撞/三模式/失效停止） | [验证计划](../testing/validation_plan.md)、[pcd_route_fix](../testing/pcd_route_fix.md) |
| [review_round2_fixes.md](review_round2_fixes.md) | 第二轮复审 4 项修正的当轮证据 | [实施报告](../migration/implementation_report.md) |
| [codex_handoff_problems.md](codex_handoff_problems.md) | 原始问题交接（含已被推翻的归因） | [pcd_route_fix](../testing/pcd_route_fix.md) |
| [field_terrain_check.md](field_terrain_check.md) | 三处坐标核对（受控洞口判据仍有效） | [pcd_route_fix](../testing/pcd_route_fix.md) |
| [tunnel_diagnosis.md](tunnel_diagnosis.md) | 洞口诊断（结论已被更正） | [pcd_route_fix](../testing/pcd_route_fix.md) |
| [height_sweep.md](height_sweep.md) | 修复前高度扫描 | [pcd_route_fix](../testing/pcd_route_fix.md)；`robot_height` 见根 [README](../../README.md) |
| [inflation_analysis.md](inflation_analysis.md) | z 膨胀核对（**z 方向解释写反、计数表作废**） | [SCAN 架构：碰撞包络参数](../architecture/scan_planner.md) |
| [mode2_field_roundtrip.md](mode2_field_roundtrip.md) | 有支撑面 Mode 2 往返对照（命令保留） | [无支撑面预览](../testing/mode2_waypoint_z_preview.md) |
| [terrain_following_history.md](terrain_following_history.md) | 地形分离原始长文与删点调查 | [地形与高度跟随](../testing/terrain_following.md) |
| [progress_history.md](progress_history.md) | 逐轮进展流水 | [进展](../migration/progress.md) |

## 使用约定

- 引用归档页时必须写明“历史记录”及取代它的当前文档，不能把其中的旧归因当作当前事实。
- 仍未解决的失败记录（例如 `log/probes/` 下未入库的 probe 日志）继续留在本地磁盘，
  归档不改变其状态；`log/`、`artifacts/`、`build/`、`install/` 仍在 `.gitignore` 内。
- 归档页的相对链接已随移动修正；地图与场景原始证据仍在 `docs/testing/maps/`、`docs/testing/evidence/` 原路径。
