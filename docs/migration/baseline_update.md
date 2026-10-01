# RM 基线更新记录

比较：82d074118fc1283273e094d0b2edbf1b90b89a15 → **7dfe71a5dbac8bd9b14cb618f0df960942611156**（main，与本地 origin/main 一致）。
用户自行拉取，本轮只读核对。SCAN2 仍为 103bce4。

| 变化 | 源码依据（RM/src 下） | 文档/迁移影响 |
|---|---|---|
| 点云按输入 stamp 查 TF，输出 base_footprint 实际坐标 | hnurm_perception/pointcloud_filter/src/pointcloud_filter_node.cpp::pointcloud_callback | 不能继续按 camera_init 点云接线；查询 timeout 0.1 s，缺 TF 时丢弃 |
| 半径与高度过滤；区域接口删除 | 同上与 params/default.yaml | 半径 0.25，z (-0.35,1.0)；删除 special_areas/transformed_special_area 端点，残留 YAML 参数不是订阅 |
| 高频 IMU odom 开启 | hnurm_fastlivo2/FAST-LIVO2/config/mid360.yaml、src/LIVMapper.cpp::imu_prop_callback | 已有 pose + 世界系线速度候选；world header、child 未填、IMU stamp；主 Odometry 仍无 twist |
| dense_map_en 开启、path stamp 更新、RViz 默认 false | 同目录 config、src、launch/mapping_mid360.launch.py | 记录云密度/时间与启动变化，不据此推定实测性能 |
| 地面分割 max_slope .75 → .99 | hnurm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/launch/segmentation_params.yaml | 更新输入处理基线 |
| STVL livox_clear decay_acceleration .2 → .1 | hnurm_navigation/params/nav2_params.yaml | 参数基线更新，不改变 Smac2D + MPPI Omni 总架构 |
| 视觉/相机/description 等 6 包删除 | 当前 package.xml 清单与 git diff | RM 当前 19 个 ROS 包；SetMode server 不再由当前 detector 提供 |
| bringup 简化到 UART；row 改 roll | hnurm_bringup/launch/bringup.launch.py、params/extrinsic.yaml | 删除旧视觉/model 启动描述，撤销拼写风险；不改原 TF |
| small1.sh 改名 full.sh | 根目录脚本 | 仍有 ROS1 bridge/Docker/外部决策历史流程，不等于默认实车入口 |
| new_msg 新增 4 份接口草稿 | hnurm_interfaces/new_msg 与 CMakeLists.txt | CMake 只匹配 msg/*/*.msg，草稿尚未生成，不能登记为活跃接口 |

## 用户澄清覆盖旧建议

1. “三维导航”就是 SCAN 当前实现，撤回完整地形/拓扑规划为必做前置的扩展。
2. 当前 RM 配置已由用户验证 RViz 2D goal 正确规划并运行；源码疑点是迁移核对项，不是现有系统不可用结论。
3. 小陀螺等部分由电控实现；仓库相关分支不能单独证明实际执行行为。
4. C 方向已认可，具体实施见 [计划](plan.md)；本轮仍不改源码、不构建运行。
