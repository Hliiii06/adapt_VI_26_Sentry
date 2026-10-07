# 真实 PCD 指定洞口与坡道：修复与使用入口

日期：2026-10-02。本轮基于本仓库 `5bbc87b` 工作树实施；下述结果为 Codex 本轮运行，
不是复述前一轮报告。用户已批准“先打通指定洞口与坡道”的有限支撑面选择方案。

## 先看结论

**CONFIRMED：不是缺少 Gazebo，也不只是把 z 膨胀调小。** 找到了并修复两处代码问题，
另为真实 PCD 增加了从入口连续跟随坡面的离线处理。现在：

| 真实 PCD 路段，Mode 1 目标链路 | 半径 / 高度 (m) | 结果 | 上升量 | 终点误差 | 实际障碍云碰撞帧 |
|---|---|---|---|---|---|
| 大坡 `(6.5,4.2) → (9,4.2)` | 0.26 / 0.25 | 到达 | 0.200 m | 0.00081 m | 0 / 2633 |
| 小洞及后坡 `(4.7,4.1) → (4.7,6.1)` | 0.26 / 0.10 | 到达 | 0.160 m | 0.00354 m | 0 / 2434 |
| 南侧洞及后坡 `(1.6,-6) → (-1.5,-6)` | 0.26 / 0.10 | 到达 | 0.200 m | 0.00831 m | 0 / 2432 |
| 同一小洞 | 0.26 / 0.25 | 拒绝、全程静止 | — | — | 22.32 s 无非零命令 |
| 同一南侧洞 | 0.26 / 0.25 | 拒绝、全程静止 | — | — | 22.31 s 无非零命令 |

独立碰撞检查使用记录的全部 odom 帧、圆柱包络和**该次运行实际使用的障碍 PCD**，
不使用规划器的“成功”日志作判据。但仍以地面分类正确为前提，不是物理安全认证。
两个拒绝场景确认 FSM 接受目标进入规划，不能把“没有发目标”算作拒绝成功。

## 你现在怎样看效果

先运行 `scripts/build.sh`。在有显示的机器上，每次只启动下面一个场景：

```bash
# 保持当前标称车高 0.25 m；RViz 2D Goal Pose 点 (9.0, 4.2)
bash scripts/run_field_route.sh large_ramp

# 降低高度，仅验证洞口规划链路；目标 (4.7, 6.1)
bash scripts/run_field_route.sh small_tunnel robot_height:=0.10

# 另一洞口及后坡；目标 (-1.5, -6.0)
bash scripts/run_field_route.sh south_tunnel robot_height:=0.10
```

这是 Mode 1，不是让模拟器抄一条预制轨迹：RViz 目标进入 FSM，由 SCAN 规划并输出给全向跟踪器。
无界面测试也是通过同一个目标话题触发；本轮**没有实际操作 RViz 窗口**。
建议先点上述目标，不要在全场任意点：此入口提供的是三个**有界地面网格**，不是全场地形图。
超出网格仍拒绝。通道边界包含测试路线的车体范围，但未实现全局轮地支撑认证。

脚本会在 `artifacts/field_routes/<场景>.<随机后缀>/` 生成地图，不覆盖原始 PCD 或旧证据。
原始文件默认为 `~/pcd_map/rmuc2026_field.pcd`，可用 `PCD_MAP_FILE` 指向同一文件的其他位置；
脚本校验原图与背景障碍云 SHA-256，地图内容改变须重新核对坐标及背景分类。
不传 `robot_height` 时仍为 **0.25 m**，不会为了演示自动缩车。没有串口/实车输出。

## 已确认的原因与对应修改

### 1. A* 下坡索引取整错误

`src/planner/path_searching/include/path_searching/dyn_a_star.h::Coord2Index` 和
`src/planner/path_searching/src/dyn_a_star.cpp::interpolateZIndexOnSearchPlane` 原先加 0.5 后
转整数，对负数是向零截断，不是向下取整。搜索坐标围绕起终点中点建立，负相对坐标很常见。
这会使下坡终点索引与搜索平面插值不一致。

**反例**：完全空的占据图中，上升 0.20 m 成功、下降 0.20 / 0.35 m 仍失败。
改成 `floor` 后，平地、上升和两个下降用例均通过。
回归代码：`src/planner/path_searching/test/search_regression.cpp`，已接入 CTest。
这证明了一处搜索缺陷，不意味着所有复杂非线性地形搜索均已解决。

### 2. 模拟雷达把薄洞顶“渲染厚了”

`src/simulator/local_sensing/src/pointcloud_render_node.cpp` 把点填充到相邻角度格，
再按距离反算球面点，还带平面插值。对低洞，返回点可能出现在原本不存在的位置。
合成低洞诊断中，原洞顶约 z=0.746，出现 z=0.625 的占据点，膨胀后达到 z=0.575；
这会挡住高度已经降为 0.10 m 的机器人，不能再解释为“窄通道一定规划不过”。

新增 `preserve_map_geometry`：发布选中静态点的原 XYZ，而非偏移后的角度格反算点；
补齐两处分支的来源索引，动态返回不套用静态来源。节点默认 false，
`sentry_sim.launch.py` 默认 true；可用 `preserve_map_geometry:=false` 对照旧行为。
这里的来源是**降采样后的地图点**，不是未处理原图；可见性仍近似，不能当真雷达仿真。
既有话题、消息类型、QoS、frame、TF 和频率未改变。

### 3. 同一 XY 同时存在底板、坡面和洞顶

原“每格最低层”方法会沿着坡底下的底板走，把真正坡面当作低障碍。
例如大坡 y≈4.2：x≈8.4 同时有 z≈0.06 底板和 z≈0.26 坡面。
仅调车高无法纠正错误的行驶表面。

`scripts/prepare_route_terrain.py` 新增**指定入口、方向和边界**的支撑层选择：

- 从已知入口高度开始，逐列跟随连续可达的上层薄表面；不再永远取最低层。
- 对垂直面内部点不当支撑，缺失/不可达格直接拒绝导出，不补出一条虚构路面。
- 通道内按选定支撑面分离坡面与障碍，上方原始点保留；通道外沿用既有背景障碍分类。
- 输出网格供规划器和模拟器查询，输出障碍 PCD 供 SCAN；原始 PCD 不改动。

这是本张量化 PCD 的离线、定向方法，不是通用地面分割：0.10 m 网格、0.04 m 原图高度间距、
0.041 m 表面容差及连续性允许值见 `_report.json`。允许值包含量化误差，**不是台阶攀爬能力**；
可能把很矮的结构归入表面，不能把该容差直接用于实车。背景区仍有旧分类的局限。
本轮核对小洞 `(x=4.4..5.0,y=5.0..5.45,z>0.4)` 的 360 个顶结构点、
南洞 `(x=0.7..1.1,y=-6.2..-5.8,z>0.2)` 的 210 个顶结构点，全部保留。
不能把这两块检查推广为整张地图分类都正确。

附带修正：`make_terrain_route.py::Ground.at` 去掉错误的半格偏移，与 C++ 网格坐标一致；
`scenario.sh` 将额外 launch 参数真正传入启动函数。旧高度扫描日志中的车高已核对，
确实为 0.10 m，不能把之前的失败全部归咎于后一个脚本问题。

## 0.25 m 为什么仍不能过洞

**CONFIRMED（当前点云模型）**：小洞附近 `(4.7,5.4)` 选中地面约 0.22 m，
顶底约 0.46 m；南洞 `(1,-6)` 地面约 0.02 m，顶底约 0.26 m。
局部净高约 **0.24 m**，小于 0.25 m，且还没留安全余量。模拟链路中拒绝有几何依据。

**UNKNOWN（真实场地）**：点云有约 4 cm 量化，实际洞净高、机器人最低姿态的完整包络、
雷达/云台是否突出尚未在现场复核。此处不能声称真实车必然不能过，也不能声称能过。
降低模型高度是定位问题的实验；如果实际车仍高 0.25 m，就不能只把 z 碰撞包络缩到 0.10 m。
原先把问题归结为“用户坐标不对”或“真实场地只有 0.14 m 可行驶起伏”的结论不再适用。

## 验证、证据与复现

本轮最终构建：9 包成功；既有 warning 尚在。无新外部依赖。
最终组合回归 `low_geometry_final`：上升/下降各 0.54596 m，终点误差 0.00522 m，
独立检查 0/4236 碰撞帧；`gap_edge`（0.44 m 缺口、0.52 m 车体直径）仍拒绝并全程静止。

```bash
python3 scripts/test_route_terrain.py  # 6 个确定性测试
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH \
  ctest --test-dir build/path_searching --output-on-failure
```

地面测试覆盖底板上坡面、洞顶不选为地面、0.15 m 孤立障碍不当坡面、垂直面、缺失格、网格坐标。
这些用例不证明任意台阶/洞顶都可正确分层。

详细本地证据（`log/` 不入库）：

- `log/probes/field_ramp_bg_final/`：最终加宽边界的大坡，`check.json` 为真。
- `log/probes/field_small_tunnel_bg_h010/`、`field_south_tunnel_bg_h010/`：两个洞口正例。
- `log/probes/field_small_tunnel_bg_h025/`、`field_south_tunnel_bg_h025/`：同图同半径高度反例。
- `log/probes/low_diagnostic/`：修复前异常占据点快照；`low_geometry_final/`：最终组合回归。
- `log/probes/terrain/`：各次实际输入地图、支撑网格和生成报告。
- 每个 probe 的 `launch_args.txt`、`launch.log`、`run.csv`、`run_cmdvel.csv` 保留参数与原始记录。

`scripts/probe_navigation.sh NAME SECONDS [launch参数...]` 只负责启动、记录、清理，
**它正常退出不代表验收通过**。Mode 1 用 `SEND_GOAL=true GOAL_X=... GOAL_Y=...`；
必须再运行独立判据，例如：

```bash
python3 scripts/check_route_probe.py \
  --csv log/probes/field_ramp_bg_final/run.csv \
  --obstacles log/probes/terrain/large_ramp_bg_obstacles.pcd \
  --goal 9 4.2 --height .25 --min-rise .18
python3 scripts/check_no_motion.py \
  --csv log/probes/field_small_tunnel_bg_h025/run_cmdvel.csv --duration 20
```

禁止把早期 `field_ramp_final` 当作净空通过证据：其原始地图边界过近，独立检查发现车体碰到
边界外未过滤地板。本轮加宽至 x=6..10 后**重新运行**，最终通过的是 `field_ramp_bg_final`。
紧凑结果与地图哈希见 [证据摘要](evidence/pcd_route_fix.json)。

## 收敛边界与下一步

1. 当前指定路线已能检验“规划出过洞/上坡路径并闭环走完”；本轮不扩建 Gazebo。
2. 下一步先在 RViz 看上述三条路线、核对实际车高和洞内净高；这是用户可直观看懂的验收。
3. 再单独安排 RM 输入/输出适配和无执行权的规划对照，不直接将仿真 cmd_vel 接电控。

尚未完成：全场多层支撑图、通用绕行后的地形选层、完整坡度/支撑/牵引约束、真实雷达与定位、
实车接入。A* 搜索高度仍含起终点线性参考，最终轨迹再按地面重投影，不能据本轮推断任意
弯曲复杂坡都能搜索成功。三个模式接口未删改，但本轮未重跑全部历史 23 场景。
仿真退出时既有 renderer 异常退出日志尚未修复；不能把正常行驶证据说成全部生命周期无缺陷。
RM main `7dfe71a`、ROS2 SCAN `103bce4` 保持只读；ROS1 既有用户修改保留。
