# 源码目录、包与依赖

基线见 [入口](../README.md)。扫描排除 build/install/log。路径相对于对应源码根目录。下表列出全部发现的 ROS package.xml，依赖栏保留架构相关直接依赖；完整构建/测试依赖以每行目录下 package.xml 和 CMakeLists.txt 为准。

## RM 目录地图

```text
VI_26_Sentry/
├── *.sh                         多终端启动组合，存在历史入口
├── src/
│   ├── hnurm_navigation/         Nav2 launch、参数、二维地图
│   ├── hnurm_bringup/            TF、UART 入口
│   ├── hnurm_fastlivo2/          嵌套 LIO、Livox、vikit
│   ├── hnurm_perception/        分割、过滤、配准、投影
│   ├── RMUL_Decision/src/       独立 BT 决策、裁判仿真
│   ├── hnurm_uart/              串口、协议、控制模式
│   ├── hnurm_interfaces/        自定义 msg/srv
│   └── hnurm_utils/                工具库（自瞄/相机/description 已清理）
└── build/install/log/            既存产物，不作为 main 运行证据
```

| package | src/ 下路径 | 职责 | 主要直接依赖/说明 |
|---|---|---|---|
| hnurm_navigation | hnurm_navigation | Nav2 编排和参数，非自研规划器 | 清单仅 ament/lint；launch 实际依赖 Nav2、RViz、slam_toolbox 等，运行依赖声明不完整 |
| hnurm_bringup | hnurm_bringup | tf_transformer、UART 启动 | rclcpp、tf2、hnurm_interfaces/uart/camera；清单还含 rm_auto_aim/rm_rune/rm_serial_driver 等本树未匹配名 |
| hnurm_uart | hnurm_uart | Twist/视觉/状态与串口 | hnurm_interfaces、hnurm_utils、rclcpp、tf2、angles |
| hnurm_interfaces | hnurm_interfaces | 消息/服务生成 | rosidl、std_msgs、sensor_msgs、nav_msgs；消息引用的 geometry_msgs 等也需核对声明完整性 |
| hnurm_utils | hnurm_utils | 日志、数学/公共工具 | ceres、Eigen、fmt、OpenCV、rclcpp |
| fast_livo | hnurm_fastlivo2/FAST-LIVO2 | LiDAR/IMU/可选视觉里程计 | livox_ros_driver2、vikit_common/ros、PCL、OpenCV、Sophus、tf2、cv_bridge |
| livox_ros_driver2 | hnurm_fastlivo2/livox_ros_driver2/livox_ros_driver2 | 雷达与 IMU 输入、自定义 msg | Livox SDK（CMake）、rclcpp、rosidl、PCL、rosbag2 |
| vikit_common | hnurm_fastlivo2/rpg_vikit/vikit_common | 视觉几何库 | OpenCV、Sophus、rclcpp；manifest/export 与 CMake 构建类型需核对 |
| vikit_ros | hnurm_fastlivo2/rpg_vikit/vikit_ros | vikit ROS 封装 | vikit_common、rclcpp、tf2、visualization_msgs |
| registration | hnurm_perception/hnurm_registration/src/registration | 先验 PCD 全局配准 | quatro、small_gicp、teaserpp、PCL、OpenMP、tf2 |
| quatro | hnurm_perception/hnurm_registration/src/Quatro | 粗配准库 | teaserpp、PCL、TBB、Eigen、OpenMP；manifest/export 与 CMake 类型需核对 |
| linefit_ground_segmentation | hnurm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation | 地面分割算法 | Eigen、PCL、pcl_ros |
| linefit_ground_segmentation_ros | hnurm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros | 分割 ROS 节点 | 上述库、rclcpp、sensor_msgs、tf2_ros |
| pointcloud_filter | hnurm_perception/pointcloud_filter | 车体半径/高度过滤，输出 base_footprint | PCL、tf2、nav_msgs；已移除 hnurm_interfaces 依赖 |
| pointcloud_to_laserscan | hnurm_perception/pointcloud_to_laserscan | 点云/激光转换组件 | laser_geometry、tf2_sensor_msgs、message_filters、rclcpp_components |
| pcd2pgm | hnurm_perception/pcd2pgm | 离线三维地图转二维栅格 | PCL、nav_msgs、tf2、Eigen |
| usb_cam | hnurm_perception/usb_cam | USB 相机驱动 | v4l、ffmpeg、image_transport、camera_info_manager；含 ROS1/ROS2 条件依赖 |
| hnurm_ul_decision | RMUL_Decision/src/hnurm_rmul_decision | 独立竞赛行为树 | behaviortree_cpp_v3、hnurm_interfaces、tf2；运行时加载 Nav2 BT 插件 |
| hnurm_referee_sim | RMUL_Decision/src/hnurm_referee_sim | 裁判数据仿真 | hnurm_interfaces、rclcpp、std_msgs |

当前共 19 个 ROS package.xml。此更新已删除 camera_calibration、hnurm_camera、armor_detector、armor_solver、hnurm_auto_aim、hnurm_robot_description；旧 manifest 对这些包的残留引用不代表包仍存在。`hnurm_interfaces/new_msg` 是新视觉/导航接口草稿，未纳入当前 CMake 的 msg/*/*.msg 生成规则，不算可用接口。

另有 **未跟踪的 `src/Sophus/` 普通 CMake 项目**，不是本轮新增，也不能默认它是 main 的受版本管理依赖。RM 的嵌套 workspace 含独立 build/install；引用旧 overlay 可能隐藏依赖问题。

## SCAN2 包

```text
SCAN-Planner-Ros2/src/
├── planner/
│   ├── plan_manage/              package 名为 scan_planner
│   ├── plan_env/
│   ├── path_searching/
│   ├── bspline_opt/
│   ├── traj_utils/
│   └── scan_planner_msgs/
└── simulator/
    ├── local_sensing/
    ├── mockamap/
    ├── map_generator/
    └── Utils/                   Go2、odom 显示、pose/waypoint 工具
```

| package | src/ 下路径 | 职责 | 主要依赖 |
|---|---|---|---|
| scan_planner | planner/plan_manage | FSM、管理器、控制器、模拟器、路径工具 | bspline_opt、plan_env、path_searching、traj_utils、scan_planner_msgs、rclcpp/rclpy、tf2；launch 还依赖仿真/Go2 包 |
| plan_env | planner/plan_env | 三维滑动占据/膨胀 | Eigen、PCL、cv_bridge、message_filters、tf2、rclcpp |
| path_searching | planner/path_searching | 碰撞段 A* | plan_env、Eigen、rclcpp |
| bspline_opt | planner/bspline_opt | B-spline、LBFGS 优化 | plan_env、path_searching、Eigen、rclcpp；LBFGS 头文件在包内 |
| traj_utils | planner/traj_utils | 多项式与可视化 | bspline_opt、Eigen、visualization_msgs、rclcpp |
| scan_planner_msgs | planner/scan_planner_msgs | Bspline/DataDisp | rosidl、builtin_interfaces、geometry_msgs、std_msgs |
| local_sensing_node | simulator/local_sensing | CPU/GPU 点云/深度渲染 | PCL、Eigen、Boost、cv_bridge、GLM；GPU 另需 OpenGL/GLEW/GLFW |
| map_generator | simulator/map_generator | 地图生成/PCD 发布 | PCL、Eigen、rclcpp |
| mockamap | simulator/mockamap | 合成场景 | PCL、Eigen、rclcpp、RViz |
| go2_description | simulator/Utils/go2_description | 四足模型与物理仿真入口 | xacro、robot_state_publisher、ros_gz_*、gz_ros2_control、controllers |
| odom_visualization | simulator/Utils/odom_visualization | 位姿/轨迹显示 | pose_utils、Armadillo、tf2、rclcpp |
| pose_utils | simulator/Utils/pose_utils | 位姿数学库 | Armadillo |
| waypoint_generator | simulator/Utils/waypoint_generator | 通用 waypoint 工具 | geometry_msgs、nav_msgs、rclcpp；不是 Mode 3 的全局搜索器 |

SCAN2 为原生 ROS2 ament/C++17；ROS1 对应工程为 catkin，不能把其 `catkin_make` 当成 SCAN2 命令。更不能由两个仓库同名库推断 ABI/消息兼容。
