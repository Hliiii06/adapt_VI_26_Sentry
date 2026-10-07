# scripts 分组登记

本页只登记**分组、用途与调用关系**，不搬迁代码、不改导航行为（整理任务边界见
[整理报告](../docs/migration/cleanup_report.md)）。ROS 包名、话题、launch 参数与启动命令均保持原样。

## 使用入口（人工启动）

| 脚本 | 用途 |
|---|---|
| `build.sh` | 隔离构建：产物只落本仓库 `build/`、`install/`，日志在 `log/ros/`，不 source 旧 overlay |
| `run_sentry_sim.sh` | 闭环仿真统一入口（`navi_mode:=1/2/3`、`robot_height`、地形参数、`start_rviz`） |
| `run_field_route.sh` | 三条已知通道入口：`large_ramp` / `small_tunnel` / `south_tunnel`（Mode 1，不发实车命令） |
| `run_waypoint_z_preview.sh` | Mode 2 无支撑面六航点诊断预览（复用 `open_loop_controller`，无速度闭环） |

## 回归与判据（失败必须返回非零）

| 脚本 | 用途 |
|---|---|
| `scenario.sh` | 场景编排：独立进程组启动、记录、判定、清理（不使用宽泛 `pkill -f`） |
| `scenario_test.py` | 记录器/触发器：显式 `--send-goal`/`--cancel-after` 驱动，`--odom-only` 用于无速度命令实验 |
| `smoke_test.sh` | 节点/话题/频率冒烟，不替代运动与碰撞结果 |
| `check_stop.py` | 取消/断流停车判据：从实际事件时刻计时 + 完整观察窗持续为零 |
| `check_no_motion.py` | “本就不该动”的判据：全程零命令 + 覆盖完整时长 + 无中间缺样 |
| `check_passage.py` | 到达/拒绝判据（`--expect reach|reject`） |
| `check_clearance.py` | 独立几何净空检查（不调用 SCAN 碰撞函数） |
| `check_terrain.py` | 高度跟随判据：按上坡/下坡/坡顶/平地分别检查方向与幅度 |
| `check_tracked_height.py` | 规划高度 vs 实际执行高度，并按实际高度重放碰撞 |
| `check_route_probe.py` | probe 结果的到达、升降与圆柱净空检查（读全部 odom 帧） |
| `test_route_terrain.py` | 支撑层选择的确定性单测（标准库，无需 ROS） |
| `test_waypoint_z_launch.py` | launch 组合的只读回归（校验执行器选择与支撑面输入被拒） |

上游 C++ 测试仍在 `src/` 内：`bspline_opt`、`plan_manage` 的既有测试，
以及本仓库新增的 `path_searching/test/search_regression.cpp`（已接入 CTest）。

## 数据准备（离线，一次生成）

| 脚本 | 用途 |
|---|---|
| `prepare_terrain_map.py` | 地面分离（背景区）：默认 `--ground-mode local`（局部低分位数 + 补洞 + 中值平滑），`global` 为旧二次曲面拟合；输出障碍云/地形表面/地面网格与 `_report.txt` |
| `prepare_route_terrain.py` | 指定入口/方向/边界的连续支撑层选择（三条通道），输出网格/障碍 PCD 与 `_report.json` |
| `make_test_maps.py` | 合成碰撞地图（封闭房间 + 带缺口隔墙） |
| `make_terrain_maps.py` | 合成 10°/20°/30° 坡道 + 解析式地面网格 |
| `make_tunnel_maps.py` | 可控洞口场景（高洞/低洞只差洞顶高度） |
| `make_lateral_slope_map.py` | 横向坡面 + 绕障场景 |
| `make_terrain_route.py` | 从地面网格生成 Mode 2 航点/Mode 3 路线（含沿线净空检查） |

## 影子接入（I1/I2，2026-10-07）

| 脚本 | 用途 |
|---|---|
| `test_shadow_entry.sh` | 影子场景编排与判据入口（`adapter_math`/`no_inputs`/`mode1_goal`/...；失败返回非零） |
| `check_shadow_graph.py` | ROS 图与门控判据：禁止话题无发布者、必需节点存在、影子输出全零/yaw 置零 |
| `fake_rm_inputs.py` | 合成 RM 输入（TF/odom/速度/云）与故障注入；**不发布速度命令** |
| `test_shadow_adapter_math.py` | 坐标/杆臂/yaw=90°/时间门控的确定性单测（无需 ROS 图） |

启动入口为 `ros2 launch sentry_scan_adapter sentry_scan_shadow.launch.py`（`start_rviz:=false` 可无界面）。
契约与回放步骤见 [影子输入契约](../docs/interfaces/shadow_input_contract.md)。

## 诊断探针（不构成验收）

| 脚本 | 用途 |
|---|---|
| `probe_navigation.sh` | 启动/记录/清理一次 probe；正常退出**不代表验收通过**，须再跑独立判据 |
| `dump_cloud.py` | 抓取一个 PointCloud2 话题首帧，离线分析 SCAN 看到的地图 |
| `inspect_field_terrain.py` | 「原始点云 / 估计地面 / 分类障碍 / 机器人包络」四层剖面核对 |
| `summarize_launch_log.sh` | 从 launch 日志提取生效配置与关键告警，生成 `*_key_log.txt` |

## 重复与不合并的理由

以下脚本名字相近、部分逻辑重叠，**用途不同，不合并**（合并会混淆判据语义或放宽退出码）：

- `check_stop.py` 要求“事件前有非零命令”，用于证明先动后停；`check_no_motion.py` 用于本就不该动的场景。
- `check_clearance.py` 只看几何净空；`check_tracked_height.py` 还比较规划/执行高度；
  `check_route_probe.py` 面向真实 PCD probe 的到达与升降。三者输入与结论口径不同。
- `prepare_terrain_map.py`（全局地面拟合，背景区）与 `prepare_route_terrain.py`
  （有界支撑层，指定通道）是两级方法，不能互相替代。
- `scenario.sh`（编排）与 `scenario_test.py`（记录/触发）职责分离，判据一律在 `check_*.py`。

**本批次不重构脚本**；若后续确实要抽象公共逻辑，另开小提交并做行为回归，
不能把导航逻辑修复藏在整理提交里，也不能为“统一”放宽判据或吞掉非零退出码。
