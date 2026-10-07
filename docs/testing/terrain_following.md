# 地形分离与高度跟随（当前实现）

2026-10-07 整理后保留的**当前实现与操作说明**。2026-10-01 的原始长文（删点根因调查、
全局二次曲面量化对比、旧结论更正）见[归档全文](../archive/terrain_following_history.md)；
指定通道的支撑层修复与证据见[真实 PCD 路线修复](pcd_route_fix.md)。

## 一、地面与障碍必须分开

地面点**绝不能**进 SCAN 的占据栅格，否则地面自身即为障碍。当前工具分两级：

| 工具 | 方法 | 适用范围 |
|---|---|---|
| `scripts/prepare_terrain_map.py` | 每格低分位数 + 稳健迭代拟合二次曲面（RMS 0.05 m），障碍 = 高于地面 ≥ 0.08 m | 背景区、开阔场地的地面/障碍分类 |
| `scripts/prepare_route_terrain.py` | **指定入口、方向和边界**的连续支撑层选择，逐列跟随上层薄表面，垂直面不当支撑，缺失格拒绝 | 三条指定通道（大坡、小洞、南洞） |

输出：`_obstacles.pcd`（喂 SCAN）、`_surface.pcd`（地形表面，**仅 RViz 显示**）、
`_ground.txt`（地面高度网格，供查询）、`_report.txt`（参数与统计）。
两者都是**离线、定向**方法，不是通用地面分割。

## 二、高度跟随链路

| 环境/输入 | z 的来源 |
|---|---|
| 运动模拟器 `go2_kinematic_sim` | `z = 地面(x,y) + body_height`（由地形推导，不抄轨迹 z） |
| Mode 3 参考路线 | `make_terrain_route.py` 从同一网格写地面 z，SCAN 再加一次 `body_height` |
| Mode 2 航点 | 生成器直接输出 `地面 + body_height`，SCAN 直接使用 |
| Mode 1 RViz 目标 | 目标处地面 + `body_height`（`grid_map.ground_grid_file` 给出后） |
| 轨迹控制点 | 优化**之后**、时间重分配之后再按新 XY 重赋地形 z（`PlannerManager::applyTerrainZToControlPoints`），越出网格即拒绝整条轨迹 |

关键参数（`sentry_sim.launch.py`）：`ground_file`（仅显示）、`ground_grid_file`（高度查询）、
`pcd_map_file`（只含障碍）、`map_offset_z`/`keep_z_min`/`keep_z_max`（**演示地图绝对高度切图，
非地面识别，不得用于实车**）、`publish_raw_cloud`。

## 三、RViz 图层默认状态

| 图层 | 默认 | 说明 |
|---|---|---|
| PCD map (obstacles / height-cut demo) | 开 | `/sentry_sim/global_cloud` |
| Sensor Cloud | 开 | `/sentry_sim/cloud` |
| Terrain surface (colour = height) | 开 | 地形高度点云 |
| Terrain surface (solid mesh) | 关 | 实心地形面，半透明，按需勾选 |
| Robot body (solid) | 开 | 半径 0.26 的圆柱，与包络同形 |
| Sentry envelope + heading | 开 | 调试层 |
| Original map / Occupancy / Inflated | 关 | 调试层，按需开 |

> 此前的坑：把点云/包络默认关掉，同时实心地形面不透明，用户只看到“一整块蓝色”。

## 四、怎么运行

```bash
# 真实场地 + 地形（缓坡，可行驶起伏约 0.14 m）
scripts/run_sentry_sim.sh navi_mode:=3 \
  reference_path_file:=$(pwd)/docs/testing/maps/field/field_slope_mode3.yaml \
  pcd_map_file:=$(pwd)/docs/testing/maps/field/rmuc2026_obstacles.pcd \
  ground_file:=$(pwd)/docs/testing/maps/field/rmuc2026_surface.pcd \
  ground_grid_file:=$(pwd)/docs/testing/maps/field/rmuc2026_ground.txt \
  map_offset_z:=0.0 keep_z_min:=-1.0 keep_z_max:=2.0 publish_raw_cloud:=false \
  init_x:=-11.75 init_y:=-7.50

# 20° 合成坡（明显；角度给定，可定量）
scripts/run_sentry_sim.sh navi_mode:=3 \
  reference_path_file:=$(pwd)/docs/testing/maps/terrain/ramp_20deg_mode3.yaml \
  pcd_map_file:=$(pwd)/docs/testing/maps/terrain/ramp_20deg.pcd \
  ground_file:=$(pwd)/docs/testing/maps/terrain/ramp_20deg_ground.pcd \
  ground_grid_file:=$(pwd)/docs/testing/maps/terrain/ramp_20deg_ground.txt \
  map_offset_z:=0.0 keep_z_min:=-1.0 keep_z_max:=2.5 publish_raw_cloud:=false \
  init_x:=0.0 init_y:=-2.0
```

命令行复算（不需要 RViz）：`scripts/scenario.sh terrain_ramp20`（合成 20° 坡）、
`terrain_field`（真实场地 Mode 3）、`terrain_field_mode1`（真实场地 Mode 1）。

## 五、必须说明的局限（不得当成爬坡验证）

1. **“z 偏差为 0”是同义反复的一部分**：模拟器与判据用同一地面网格，只证明链路一致
   （网格加载、查询、z 传播、三模式高度语义）；有信息量的是“上升量 = `tan(角度) × 距离`”。
2. **没有动力学**：无轮地接触、牵引、打滑、质量、坡度阻力；30° 通过只说明运动学积分能跟随高度。
3. **地图里的“地面”是地图自己表达的地面**：穹形起伏可能是真实地貌也可能是建图伪影，需场地 owner 确认。
4. **A* 搜索高度仍含起终点线性参考**（`applyLinearZReference`），最终轨迹再按地面重投影；
   不能据此推断任意弯曲复杂坡都能搜索成功。
5. **不做地形规划**：无台阶/悬崖/可通行性分析、无跨层全局搜索，与“三维导航 = SCAN 当前实现”一致。
6. **RViz 图形交互本环境无法验证**（创建不了 OpenGL 上下文），显示效果需用户确认。

## 六、被取代的旧结论

- “x=−6 走廊净空 1.187 m”是绝对高度删点造成的假象：该走廊实际有 3822 个障碍点。
- “真实场地只有 0.14 m 可行驶起伏、洞口过不去是坐标/矮坎问题”已被
  [真实 PCD 路线修复](pcd_route_fix.md)取代：大坡 H=0.25 m 通过，两洞 H=0.10 m 通过、
  H=0.25 m 拒绝（局部净高约 0.24 m）。
