# 地形分离与高度跟随

日期：2026-10-01。起因：用户验收 RViz 时反馈"观察全向机器人通过有坡度的地形不太明显"。

## 一、根因：不是 RViz 的问题，是地形被删掉了

排查发现两件事同时成立：

1. **运动模拟器的 z 恒定**。`mode1_lateral`/`mode3_path`/`spin` 全程 z = 0.1250，
   变化 **0.000000 m**——机器人是在一个平面上滑行。
2. **演示地图把地面整层删了**。上一版按绝对高度删点，631508 点只留 177762 点（28.1%），
   保留下来的全是障碍物，地面点一个不剩。

而场地本身是有起伏的。用每格最低点看地面（0.5 m 网格）：

| 位置 | 地面高度 |
|---|---|
| 外圈 y≈±8 m | **-6 cm** |
| 中圈 y≈±4 m | **+6 cm** |
| 内圈 y≈±1.5 m | **+10~14 cm** |
| 正中心 | 另有 **+30~60 cm** 的凸起结构（同格内仍有 -6 cm 地面点，说明是立在地面上的结构） |

地面是一个**平滑穹形**（沿 y 方向，中心高边缘低，总落差约 0.17 m），
0.30 m 的绝对阈值正好把 0–30 cm 整层删掉——**缓坡和全部低矮场地元素一起消失**。

## 二、方法：按"离地面多高"分类，而不是按绝对高度

`scripts/prepare_terrain_map.py`：

1. 按 0.5 m 网格取每格 z 的 5 分位数，作为地面候选；
2. 用**稳健迭代**拟合二次曲面 `z = f(x,y)`（反复丢弃残差大的格再拟合）。
   这样被结构占满的格（例如中央结构）会被当作离群点剔除，而不是把结构顶面误当成地面。
   实际收敛：使用 1911/1930 格，**RMS 残差 0.050 m**；
3. 拟合结果 `z = 0.1049 - 0.000029x² - 0.002635y²`，与观测到的穹形一致；
4. **障碍 = 高于拟合地面 ≥ 0.08 m 的点**，其余算地面点。

输出四个文件：`_obstacles.pcd`（喂给 SCAN）、`_surface.pcd`（地形表面，**仅 RViz 显示**）、
`_ground.txt`（地面高度网格，供查询）、`_report.txt`。

分类正确性核对：开阔区的地面是一层**尖锐薄面**（某格 p90−p10 = 0.000 m），
所以 8 cm 阈值不会把地面噪声误判成障碍；而障碍占比高的格子经查是地面与结构真实混在一起。

## 三、量化：新方法保住了什么

| | 上一版（绝对删点） | 本方法 |
|---|---|---|
| 入库点数 | 177762（删掉 71.9%） | 631508 全部保留 |
| 障碍点 | 177762 | **310659** |
| 其中 z < 0.30 m（上一版全删） | 0 | **132897** |
| 地面点 | 全部被删 | 320849（用于高度查询） |
| 演示走廊 x=−6 的障碍点 | 0（被判为空） | **3822**（z 0.06~0.34） |

**必须说明的后果**：上一轮报告里"x=−6 走廊净空 1.187 m"是删点造成的假象——
那条走廊实际上有 3822 个障碍点。旧地图把真实结构当成了空地。
本轮不再对该走廊的旧结论作数。

## 四、高度跟随

| 环节 | 做法 |
|---|---|
| 运动模拟器 | `ground_grid_file` 给出后，`z = 地面(x,y) + body_height`。**由地形推导**，不是抄轨迹的 z——地面机器人的 z 是地形的结果，不是被控量 |
| Mode 3 参考路线 | `make_terrain_route.py` 从同一网格生成路线 z（只给地面高度），SCAN 再加 `body_height`，只加一次 |
| Mode 2 航点 | 同一生成器，直接输出 `地面 + body_height`（Mode 2 的 z 被 SCAN 直接使用） |
| Mode 1 RViz 目标 | 新增：`grid_map.ground_grid_file` 给出后，目标高度取**目标处地面 + body_height**，而不是初始位姿的 z |

生成器同时做沿线净空检查，选到会撞障碍的路线会报警。

## 五、验证结果

### 合成坡道（`docs/testing/maps/terrain/`，角度给定，可定量）

| 场景 | 坡度 | 期望上升 | 实测 z 变化 | 与地形偏差 | 结果 |
|---|---|---|---|---|---|
| `terrain_ramp10` | 10° | 0.529 m | **+0.529 m** | 0.0000 m | 通过 |
| `terrain_ramp20` | 20° | 1.092 m | **+1.092 m** | 0.0000 m | 通过 |
| `terrain_ramp30` | 30° | 1.732 m | **+1.732 m** | 0.0000 m | 通过 |

上升量与 `tan(角度) × 3 m` 完全一致，反向下降采样点 0 个。

### 真实场地（`docs/testing/maps/field/`）

| 项 | 值 |
|---|---|
| 路线 | (−11.75, −7.50) → (−11.00, −1.75)，长 5.80 m |
| 地面高度 | −0.046 → +0.094 m，变化 **0.140 m** |
| 机器人 z | 0.079 → 0.219 m（= 地面 + 0.125） |
| 与地形偏差 | 0.0000 m |
| 沿线障碍净空 | 0.714 m |

**真实场地的可行驶起伏只有 0.140 m / 5.80 m ≈ 1.4°**。中央那个 0.6 m 的结构是立在
地面上的障碍，不是可行驶坡道。所以真实场地的坡是**温和**的；要看明显的坡度必须用合成坡道。

## 六、必须说明的局限（不得当成爬坡验证）

1. **偏差 0 是同义反复的一部分**：模拟器与判据用的是同一个地面网格，所以"z 偏差为 0"
   只能证明**链路正确**（网格加载、查询、z 传播、Mode 1/2/3 的高度语义一致），
   **不能**证明机器人能爬坡。真正有信息量的是"上升量等于 tan(角度)×距离"。
2. **没有动力学**：没有轮地接触、牵引力、打滑、质量、坡度阻力。30° 能跑通只说明
   运动学积分能跟随高度，**不代表实车能爬 30°**。
3. **地图里的"地面"是地图自己表达的地面**：那条沿 y 的穹形可能是真实场地起伏，
   也可能是建图过程中的回环/曲率伪影。这需要场地 owner 确认；仿真一律以地图为准。
4. **轨迹 z 仍是线性插值**：SCAN 的 `applyLinearZReference` 在局部起终点之间线性插值 z。
   地形曲率大时，碰撞查询用的 z 会与真实地面有偏差（本场地起伏 0.17 m，影响有限）。
   若要严格一致，需要让局部轨迹 z 也跟随地面——本轮**未实现**。
5. **不做地形规划**：没有台阶/悬崖/可通行性分析，没有自由三维，没有跨层全局搜索。
   这与用户此前确认的"三维导航 = SCAN 当前实现"一致。
6. **RViz 图形交互仍未验收**：本环境创建不了 OpenGL 上下文，所有验证在
   `start_rviz:=false` 下完成。地形表面、颜色映射、机器人爬坡的**显示效果需要用户确认**。
7. 上一版按绝对高度删点的路径仍然保留（`keep_z_min`/`map_offset_z` 参数未删），
   但已明确标注为演示用途；`low_obstacle_cut` 场景继续复现它的危害。

## 七、在 RViz 里怎么看

```bash
cd /home/hzq/nav/adapt_VI_26_Sentry

# 真实场地 + 地形（缓坡，起伏 0.14 m）
scripts/run_sentry_sim.sh navi_mode:=3 \
  reference_path_file:=$(pwd)/docs/testing/maps/field/field_slope_mode3.yaml \
  pcd_map_file:=$(pwd)/docs/testing/maps/field/rmuc2026_obstacles.pcd \
  ground_file:=$(pwd)/docs/testing/maps/field/rmuc2026_surface.pcd \
  ground_grid_file:=$(pwd)/docs/testing/maps/field/rmuc2026_ground.txt \
  map_offset_z:=0.0 keep_z_min:=-1.0 keep_z_max:=2.0 publish_raw_cloud:=false \
  init_x:=-11.75 init_y:=-7.50

# 20° 合成坡（明显）
scripts/run_sentry_sim.sh navi_mode:=3 \
  reference_path_file:=$(pwd)/docs/testing/maps/terrain/ramp_20deg_mode3.yaml \
  pcd_map_file:=$(pwd)/docs/testing/maps/terrain/ramp_20deg.pcd \
  ground_file:=$(pwd)/docs/testing/maps/terrain/ramp_20deg_ground.pcd \
  ground_grid_file:=$(pwd)/docs/testing/maps/terrain/ramp_20deg_ground.txt \
  map_offset_z:=0.0 keep_z_min:=-1.0 keep_z_max:=2.5 publish_raw_cloud:=false \
  init_x:=0.0 init_y:=-2.0
```

RViz 里重点看：

- **Terrain surface (colour = height)**：地形表面，按高度着色（rainbow）——缓坡会显示成渐变色带；
- **Sentry envelope + heading**：蓝色包络圆柱 + 橙色机头箭头，随地形一起升降；
- **Occupancy / Inflated Occupancy**：SCAN 实际看到的占据与膨胀（**不含地面**，地面只走 surface 话题）。

命令行复算（不需要 RViz）：

```bash
scripts/scenario.sh terrain_ramp20     # 合成 20° 坡
scripts/scenario.sh terrain_field      # 真实场地缓坡
```
