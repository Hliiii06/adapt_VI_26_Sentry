# Mode 2：关闭支撑面，只由 XYZ 航点引导高度

2026-10-06，按用户最新要求实施。此前的[有支撑面往返](../archive/mode2_field_roundtrip.md)保留为对照（已归档），
不再作为这次实验的启动入口。

## 启动与观察

在仓库根目录运行（本轮已构建）：

```bash
bash scripts/run_waypoint_z_preview.sh
```

Mode 2 自动开始，不需要 RViz 点目标。关闭本次实验用终端 **Ctrl-C**。
不要与同一 ROS domain 的另一个 `/sentry_sim` 实验同时运行。
无界面运行可追加 `start_rviz:=false`。以后修改 C++ 后先运行 `bash scripts/build.sh`。

RViz 中观察 `PCD map`、`Sentry envelope + heading (debug)` 和规划轨迹。
本模式不启动运动积分器，因此其 `Robot body (solid)` 和 `Robot Path` 没有数据，可取消勾选；
用规划器发布的圆柱包络观察模型。没有支撑面点云或地形实体网格。
本轮未验证 GUI 实际观感。

## 六个航点

配置：[south_tunnel_waypoint_z_mode2.yaml](maps/field/south_tunnel_waypoint_z_mode2.yaml)。
坐标是 `world` 系米制**机体参考中心**，不是地面坐标。初始位置 `(1.90, -6.00, 0.170)`。

| 顺序 | x | y | z | 用途 |
|---|---:|---:|---:|---|
| 1 | 0.20 | -6.00 | 0.170 | 穿过南侧洞口，到坡脚 |
| 2 | -0.80 | -6.00 | 0.370 | 上升到平台入口 |
| 3 | -1.70 | -6.00 | 0.370 | 平台折返 |
| 4 | -0.80 | -6.00 | 0.370 | 返回坡顶 |
| 5 | 0.20 | -6.00 | 0.170 | 下降到坡脚 |
| 6 | 1.90 | -6.00 | 0.170 | 再次穿洞，返回起点 |

航点负责引导，SCAN 仍执行 A*、轨迹优化和三维体素碰撞检查，不是强制逐点传送。
`fsm.waypoint_reached_distance=0.10`，替代本实验中原来过早切换的 0.5 m；
新增参数的默认值仍为 0.5 m，不改变原有场景默认行为。
本 YAML 同时设置已有参数 `fsm.thresh_no_replan=0.5`，让临近终点的现有轨迹完成，
避免不断请求短于规划器 0.2 m 最短起终点距离的新轨迹。未放松碰撞或动力学阈值。

## “关闭支撑面”具体意味着什么

- 直接加载原始 `~/pcd_map/rmuc2026_field.pcd`，无地面分割、无支撑层选择、无地图平移，
  高度窗口为 `[-10000,10000]`，不通过切掉地面来获得通路。仍有原有地图降采样和模拟雷达可见性近似。
- `ground_grid_file`、`ground_file` 均为空；`waypoint_z_preview` 会拒绝非空输入，防止混入支撑面。
- 规划器不调用地形网格赋 z 的分支，而沿用航点/局部起终点高度参考与线性 z 逻辑。
- 默认速度积分模拟器在没有网格时 **z 不变**，不能演示此实验。因此单独复用已有
  `open_loop_controller`，按 SCAN 发布的三维 B 样条生成模拟 odom；不启动速度跟踪器或运动积分器。
  不发布 `cmd_vel`，不连接 RM，也不改变默认 `closed_loop` 模式。
- 朝向保持初始值，返程不需要转头。这不是轮子受力、坡面接触、牵引力或打滑仿真。

**尺寸和离地间隙必须一起理解**：半径 0.26 m、高度 0.10 m，仅为诊断圆柱。
平地/平台参考中心约为地面 +0.15 m，因此底部约离地 0.10 m；不是贴地的 `地面+H/2`。
这让原始地面保留为障碍，同时验证一条有空间净空的三维路径。
结果不能推广为 H=0.25 m 实车过洞，也不能证明全向底盘实际爬坡。
若要强制轮地接触，需要另外提供地形接触机制，单凭航点 z 无法保证。

预览复用的开环执行器不实现闭环跟踪器的任务授权/取消闩锁。
不要把 `planning/reset` 当成可靠停车手段，不运行实车，不据此验收取消安全性。

## 本轮证据

**CONFIRMED，无界面实测**：`log/probes/waypoint_z_raw_six_final/`。

- 9 个包构建成功；启动组合测试 `scripts/test_waypoint_z_launch.py` 通过：
  两种支撑面输入均被拒绝，预览只启用开环执行器，默认入口仍使用闭环执行器与积分器。
- 4200 个 odom 样本、42.002 s，路径约 7.073 m；六个航点按顺序进入 0.11 m 三维邻域。
- z 范围 `[0.1700,0.3732]` m，上升 0.20318 m、下降 0.20320 m；终点 XY 误差 0.00405 m。
- 独立使用**原始 PCD 全部点**回放圆柱包络：0/4200 帧碰撞。
  FSM 正常从 `EXEC_TRAJ` 回到 `WAIT_TARGET`，未进入急停。
- 仅验证采样位置净空，非连续时间碰撞证明；无速度闭环/接触动力学验证，未重跑全部历史场景。

独立复查命令：

```bash
python3 scripts/check_route_probe.py \
  --csv log/probes/waypoint_z_raw_six_final/run.csv \
  --obstacles "$HOME/pcd_map/rmuc2026_field.pcd" \
  --goal 1.9 -6 --height .1 --min-rise .18 --min-drop .18
```

原始 PCD SHA256：`ebc0add17e2a229abed880926f4f01baf89ca9a8fe7a34d80b8b2b07b986adc7`。
启动脚本检查哈希，防止在不同地图上误用固定航点。

失败记录也保留：首个 `waypoint_z_raw_six` 因空值 launch 参数语法失败；
`waypoint_z_raw_six_v2` 使用旧 0.5 m 切换距离、0.1 m 终点免重规划距离，
过早下降导致 43 帧碰撞，近终点进入反复重规划/急停，**不能算通过**。
其日志引用的 YAML 路径相同但文件已更新，旧参数以本说明为准。

## 代码入口

- `scripts/run_waypoint_z_preview.sh`：地图校验与实验参数。
- `src/planner/plan_manage/launch/sentry_sim.launch.py::_setup`：禁用支撑面、选择唯一执行器。
- `src/planner/plan_manage/src/scan_replan_fsm.cpp`：航点切换距离参数，默认行为保持。
- `src/planner/plan_manage/src/open_loop_controller.cpp::publishOdom`：已有样条位姿预览执行器，本轮未修改。
- `scripts/scenario_test.py --odom-only`：记录不产生速度命令的实验，不把缺少 cmd_vel 误作超时。
