# Codex 交接：真实 PCD 下过不去斜坡与洞口

日期：2026-10-01。交接人：本会话的适配执行方。仓库 `adapt_VI_26_Sentry`，分支 `main`。

## 0. 一句话

**解析拟合出来的合成坡道/洞穴能过，换成真实场地 PCD 就不行。**
用户怀疑是"机器人 z 高度"或"目标点 z 高度"的问题。本文把已知事实、未解问题、
我做过的错误判断和下一步建议实验全部列出，供 Codex 接手。

## 1. 目标与验收标准

用户目标：全向哨兵（半径 0.26 m，标称高 0.25 m）能在真实场地 PCD 上
**通过中央洞口（含洞口后的斜坡）** 并 **上两侧斜坡**。
用户已确认场地语义：中央区域**只能**由左右两侧的坡、或经过洞口再上坡到达；
直接通往中央需要经过台阶，全向模型无法抵达。

用户给的地形坐标（用户已说明：该坐标在洞口中心**偏左**，洞口两边是高地）：

| 地形 | 坐标 | 对称位置 |
|---|---|---|
| 洞口（后有斜坡通往中央） | (0.98, −6.18) | 对称处 |
| 洞口+斜坡（小型） | (4.70, 5.22) | 对称处 |
| 大斜坡 | (7.38, 4.22) | 对称处 |

## 2. 已经能做到的（CONFIRMED，本轮实测）

| 项 | 结果 | 证据 |
|---|---|---|
| 合成 10°/20°/30° 上坡 | 上升 0.529 / 1.092 / 1.732 m，与 `tan(角)×3m` 一致 | `scripts/scenario.sh terrain_ramp10/20/30` |
| 合成下坡 | 下降 1.092 m | `terrain_down` |
| 合成跨越坡顶 | 升 0.728 / 降 0.728，峰值在路程 47% | `terrain_crest` |
| 横向坡面绕障：规划高度 vs 执行高度 | 212 采样点最大偏差 **0.0096 m** | `terrain_lateral` |
| 受控高洞（洞顶 0.45 m） | 通过（到 y=6.95） | `terrain_tunnel_high` |
| 受控低洞（洞顶 0.20 m） | 拒绝（停在 y≈1.9） | `terrain_tunnel_low` |
| 真实场地平地上的 Mode 1 | 到目标 0.004 m（(−11.50,−7.00)→(−11.00,−2.00)） | 本会话实测 |
| **真实场地洞口通道 (2.0,−7.2)→(2.0,−5.5)** | **到目标 0.01 m，H=0.25 m** | `log/scenarios/probe_0.125_0.125*` |
| 取消/竞态/越界等安全项 | 全部通过（含注入延迟旧授权） | `cancel_race`、`goal_out_of_grid` |

**注意**：真实场地并非完全不可用——上面第 7、8 行说明平地与"洞口右侧通道"是能走通的。

## 3. 用户的核心观察与假设

> "拟合出来的那个斜坡机器人是能够正常通过的，但是一换到 .pcd 中又不行了。
>  我感觉要不然就是机器人的 z 高问题，要不然就是发的目标点的 z 高问题。"

## 4. 对"z 高度"假设的核查结果：**目前不成立**

### 4.1 目标点 z —— 确实由地面网格推导（Mode 1）

日志原文（真实场地，Mode 1，目标 (2.00,−5.50)）：

```
[scan_planner_node]: Set RViz goal height from initial body_pose z: 0.105
[scan_planner_node]: Terrain following: goal (2.00, -5.50) ground z=0.020
                     -> goal z=0.145 (replacing rviz_goal_height 0.105)
```

RViz 发的目标 z 会被**替换**为 `地面(目标) + body_height`。所以"目标点 z 没设对"
在这条路径上**不成立**。（`scan_replan_fsm.cpp` 里 Mode 1 的 `goal_z` 计算，
以及 Mode 3 由 `make_terrain_route.py` 写入、FSM 再加 `body_height`。）

### 4.2 机体 z —— 与规划器同源

- 模拟器：`z = ground(x,y) + body_height`（`go2_kinematic_sim.cpp`）
- 规划器：优化与时间重分配**之后**按新 XY 重新赋地形 z（`applyTerrainZToControlPoints`，
  `planner_manager.cpp`）
- 实测一致性：`terrain_lateral` 场景下规划高度 vs 执行高度最大偏差 0.0096 m

### 4.3 两个地面模型在真实场地路线上的差异很小

验证过的路线 (2.00,−7.20)→(2.00,−5.50) 沿线：

| | 局部低分位数网格 | 全局二次曲面 | 差 |
|---|---|---|---|
| 起点 y=−7.20 | −0.020 | −0.023 | +0.003 |
| y=−6.15（洞口） | +0.014 | +0.012 | +0.002 |
| 终点 y=−4.65 | +0.054 | +0.053 | +0.001 |

沿线差异 ≤ 0.010 m。**所以在这条路上 z 不是问题。**

### 4.4 但有一个**未被排除**的 z 隐患（见 P2）

局部地面网格有 **32/2040 格 > 0.20 m，最高 0.540 m** —— 那是**爬到了结构顶上**。
若机器人行驶到这些格附近，机体 z 会跟着抬到 0.54+0.125，碰撞查询带整体上移，
**可能进入结构内部造成误判阻挡**。验证过的路线不在这些格里，但长距离运行可能撞上。

## 5. 未解决的问题（按嫌疑排序）

### P1（最高嫌疑）真实 PCD 的障碍分类可能把"可行驶的坡面/洞口地面"判成障碍

分类规则（`scripts/prepare_terrain_map.py`）：相对局部地面高度 ≥ `--obstacle-height`
（默认 0.08 m）即判为障碍。

- 已知：洞口 (0.98,−6.18) 半径 0.26 m 内有一个 **0.24 m 的密集面**（局部地面 0.013 m）。
  用户已说明那是**洞口两侧的高地**，不是台阶。
- **未知**：洞口**内部**的地面、洞口后面的**斜坡面**是否也被判成了障碍？
  若是，机器人当然进不去。
- 需要的实验：把「原始点云 / 估计地面 / 分类障碍 / 机器人包络」逐处叠加核对
  （工具已具备：`scripts/inspect_field_terrain.py`，输出样例
  `docs/testing/field_terrain_overlay.txt`）。

### P2 局部地面估计会爬到结构顶上

同上 4.4。32/2040 格。`compute_local_ground()` 取每格 5% 分位数 + 补洞 + 中值平滑；
**没有"不得显著高于周围"的约束**，因此结构顶面可能成为"地面"。
后果：机体 z 抬高 → 碰撞查询带上移 → 误判。
建议：给地面估计加连续性/连通性约束，或限制单格相对邻域的最大抬升。

### P3 规划器在窄通道里过不去（`A-star failed`）

- 受控低洞在 `robot_height:=0.10` 时，**几何上中心自由通道已有 0.30 m**
  （[−0.15,+0.15]），但规划器仍 `A-star failed` **2610 次**，机器人停在同一位置。
- 即：几何通了，规划器仍过不去。与高度无关。
- 相关参数：`manager.planning_horizon` 3.5 m（局部目标前瞻）、A* 回弹搜索。
- 需要的实验：把受控洞穴的门宽从 0.9 m 扫到 1.6 m，找出规划器能过的**门宽阈值**。

### P4 真实场地 Mode 1 不做全局搜索

Mode 1 从当前位姿直接生成多项式参考，靠局部优化绕障。
洞口这种需要"横移对准"的通道，单点目标基本不可行（上游行为，非本适配引入）。
真实场地应改用 Mode 3 参考路线，但路线必须在**与实际一致的代价地图**上生成。

### P5 合成地图与真实 PCD 不可直接类比（很可能是用户观察的主因）

| | 合成坡道/洞穴 | 真实场地 PCD |
|---|---|---|
| 几何 | 脚本按解析式生成 | 扫描数据 |
| 地面 | 解析式，精确 | 局部低分位数估计，有误差 |
| 障碍 | 规则平面墙/顶 | 噪声、遮挡、密度不均、阈值效应 |
| 尺寸 | 1.2 m 走廊、0.9 m 门 | 不规则、部分区域极窄 |

**合成能过不能推出 PCD 能过**。要定位差异，必须做"同一条几何、两种数据"的对照（见 7.1）。

## 6. 我（执行方）犯过的错误 —— 供 Codex 避免重复

1. **断言"上游没有 `obstacles_inflation_z_up/down`"** —— 错的。
   实际在 `/home/hzq/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml:48-49`，
   **up=0.1 / down=0.4**。我 cd 到了 `/home/hzq/nav`（该路径不存在）且 grep 带
   `2>/dev/null` 吞掉了错误，把空输出当成"不存在"。**教训：路径类命令不要屏蔽 stderr。**
2. **断言"洞顶不是阻挡原因"** —— 基于一个只查"高于地面 0.25 m 的最低点"的测量，
   漏掉了 **0.24 m 的密集面**。修正后：该处确实有机体高度带内的材料。
3. **判据写错**：算出"中心可放置区间"后又要求"宽度 ≥ 2×半径"，半径重复扣了一次。
   正确判据是该区间**非空**。
4. **探测设计缺陷两次**：一次目标点落在阻挡区内（规划器直接拒绝，四种配置都不动，
   无区分度）；一次路线本来就在自由侧（三种配置结果相同，无区分度）。
5. **RViz 图层误关**：把 PCD 点云层默认关掉，同时新加的地形实体网格不透明，
   导致用户"看不到点云，只有一整块蓝色"。已修。

## 7. 建议 Codex 优先做的实验

### 7.1 「同一条几何、两种数据」（最能把 P5/P1 分开）

把合成坡道/洞穴场景的**解析地面网格**替换为**用真实 PCD 加工流程（局部低分位数估计）
得到的网格**，障碍云也换成同一流程的产物，其它不变。

- 若此时退化 → 问题在**数据处理**（地面估计/障碍分类），不在几何、不在规划器。
- 若无退化 → 问题在**真实几何本身**（通道太窄/净空不够）。

### 7.2 在真实场地的**坡道**上复验 z 一致性

`terrain_lateral`（横向坡 + 绕障，比较规划 z 与执行 z）目前只在**合成地形**上做过。
应在真实场地选一段坡道，录 `*_planned.csv`（规划轨迹采样，记录器已支持）与 `body_pose`，
比较规划高度与执行高度的偏差。

### 7.3 逐处叠加核对

对三处坐标 + 洞口实际中心，用 `scripts/inspect_field_terrain.py` 输出
「原始点云 / 估计地面 / 分类障碍 / 包络」剖面，**人工判断分类是否把可行驶面判成了障碍**。

### 7.4 窄通道门宽扫描

受控洞穴门宽 0.9 → 1.6 m，找规划器能通过的阈值（对应 P3）。

### 7.5 地面估计防"爬顶"

给 `compute_local_ground()` 加约束：单格地面相对邻域中位数的抬升不超过某阈值；
或对地面网格做连通性检查，剔除与主体不连通的"孤岛高台"（对应 P2）。

## 8. 关键文件与命令

**代码**
- `scripts/prepare_terrain_map.py` —— 地面估计（`--ground-mode local|global`）与障碍分类（`--obstacle-height`）
- `scripts/make_tunnel_maps.py` / `make_lateral_slope_map.py` / `make_terrain_maps.py` —— 合成场景
- `scripts/make_terrain_route.py` —— 由地面网格生成 Mode 2/3 路线（含机体高度带净空检查）
- `scripts/inspect_field_terrain.py` —— 四层叠加剖面（P1 的核对工具）
- `scripts/check_tracked_height.py` —— 规划高度 vs 执行高度 + 实际高度碰撞重放
- `src/planner/plan_manage/src/planner_manager.cpp` —— `applyTerrainZReference` / `applyTerrainZToControlPoints`
- `src/planner/plan_manage/src/go2_kinematic_sim.cpp` —— 机体 z 跟随
- `src/planner/plan_env/src/grid_map.cpp` —— `rebuildInflationOffsets()`（z 膨胀）

**launch 参数**（`sentry_sim.launch.py`）
`navi_mode`、`init_x/init_y`、`pcd_map_file`、`ground_file`、`ground_grid_file`、
`map_offset_z`、`keep_z_min/max`、`publish_raw_cloud`、
`robot_height`（唯一高度旋钮，派生 body_height 与 z 包络）、
`inflation_z_up/down`、`robot_radius`、`start_rviz`、`reference_path_file`、`keypoints_file`

**命令**
```bash
scripts/build.sh
scripts/scenario.sh terrain_ramp20 | terrain_down | terrain_crest
scripts/scenario.sh terrain_tunnel_high | terrain_tunnel_low
scripts/scenario.sh terrain_lateral | cancel_race | goal_out_of_grid
```

**真实场地数据**
`docs/testing/maps/field/rmuc2026_local_{obstacles.pcd,surface.pcd,ground.txt}`

## 9. 相关文档

- [真实场地地形核对](field_terrain_check.md)
- [z 向膨胀参数核对与实测](inflation_analysis.md)
- [降低高度做可通行性排查](height_sweep.md)
- [地形与高度跟随](terrain_following.md)
- [Codex 第二轮复审修正](review_round2_fixes.md)
