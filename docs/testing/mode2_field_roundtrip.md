# Mode 2：真实南侧洞口与坡道往返

本页保留为有支撑面的历史对照。用户最新要求的**关闭支撑面、六个 XYZ 航点**实验见
[无支撑面 Mode 2 预览](mode2_waypoint_z_preview.md)，不要使用本页命令代替。

日期：2026-10-06。使用已验证的同一条南侧通道，行程为：
**洞外出发 → 穿洞 → 上坡 → 平台折返 → 下坡 → 再穿洞 → 返回洞外**。
不是把三个相隔的地形区域串成未经验证的全场路线。

## 启动

在仓库根目录运行（使用已有构建，无新增 C++ 修改）：

```bash
bash scripts/run_field_route.sh south_tunnel \
  navi_mode:=2 robot_height:=0.10 init_z:=0.07 \
  keypoints_file:="$PWD/docs/testing/maps/field/south_tunnel_roundtrip_mode2.yaml"
```

Mode 2 收到初始 odom 后**自动开始**，无需点击 RViz 2D Goal。
底层通用入口仍打印 Mode 1 的目标提示，忽略该提示即可；本命令末尾的参数切换为 Mode 2。
不要同时运行其他使用相同 ROS domain 和 `/sentry_sim` 命名空间的实验。

半径 0.26 m、高度 **0.10 m，仅为诊断模型**。不要把航点文件直接用于 H=0.25 m：
z 是机体中心高度，且此前同一洞口 H=0.25 m 未通过。没有实车输出。

## 航点及含义

配置：[south_tunnel_roundtrip_mode2.yaml](maps/field/south_tunnel_roundtrip_mode2.yaml)。

| 位置 | x | y | 机体中心 z（m） |
|---|---:|---:|---:|
| 初始位姿，不写入航点列表 | 1.60 | -6.00 | 0.070 |
| 航点 1：平台折返引导点 | -1.50 | -6.00 | 0.270 |
| 航点 2：洞外终点 | 1.60 | -6.00 | 0.070 |

两个航点之间由 SCAN 生成连续轨迹，不是只显示这两个点。去程向负 X，返程向正 X；
默认 `yaw_mode=hold`，车头不必转 180°，全向模型可以保持朝向反向平移。
当前 FSM 允许距中间航点 0.5 m 内切换，因此不承诺精确到达折返点再停车。

## 本实验能证明什么

保留南侧通道的支撑面网格与障碍分层。规划器仍会按网格重赋局部轨迹高度，
运动模拟器仍按网格更新 z。因此本实验用于观察 **Mode 2 自动串联、过洞和坡道往返**，
不是“去掉支撑面，只靠 XYZ 航点”的对照，也不是物理爬坡验证。

首版 6 个较密航点在第一个短段出现动态可行性拒绝，随后重规划尝试越界并停止，
没有完成行驶。日志保留于 `log/probes/south_roundtrip_mode2/`。
最终改为两个稀疏目标，没有放松碰撞、加速度或边界检查。
最终测试入口为 `log/probes/south_roundtrip_mode2_sparse/`；逐帧碰撞检查以实际使用的
`log/probes/terrain/south_tunnel_bg_obstacles.pcd` 为准，仍以点云分类正确为前提。

## 本轮无界面验证结果（CONFIRMED）

- 实际折返位置约 `(-1.3064, -6.0, 0.27)`，符合中间航点提前切换语义。
- 返回位置约 `(1.5950, -6.0, 0.07)`，距终点 0.00497 m。
- 路径长度 5.808 m，上升和下降各 0.200 m，朝向保持不变。
- 6202 个 odom 样本、62.01 s 记录；按实际障碍云回放，圆柱碰撞帧为 0。
- 另检查了确实到达平台侧 `x < -1.0`、行程超过 5 m，再返回洞外；不是“原地不动也在终点”的假通过。
- 使用已有安装二进制，未修改 C++、未重新构建。本轮未操作 RViz GUI。

判据命令：

```bash
python3 scripts/check_route_probe.py \
  --csv log/probes/south_roundtrip_mode2_sparse/run.csv \
  --obstacles log/probes/terrain/south_tunnel_bg_obstacles.pcd \
  --goal 1.6 -6 --height .1 --min-rise .18 --min-drop .18
```
