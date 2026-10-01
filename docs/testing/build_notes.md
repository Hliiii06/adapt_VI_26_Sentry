# 构建与依赖说明（本轮不执行）

用户已明确将当前任务收敛为架构熟悉和文档。**本轮没有 main 的构建验证；用户已确认当前 RM 配置实车导航成功，两者不要混淆。**

## 当前环境与历史尝试

静态/只读检查发现 `/opt/ros/humble`、`/usr/bin/colcon` 可用。上一轮在用户收敛任务前启动过两个隔离 colcon 尝试，均停在第一个 package 前缀环境准备阶段，没有进入可确认的编译完成状态。RM 那次使用的是 feature/odin，不能代表 main。

收到停止构建要求后，已终止本任务的两个 colcon 进程，退出码均为 143。保留 `artifacts/rm/`、`artifacts/scan/` 的少量未完成日志/前缀文件供追溯；没有删除原工程产物，也没有运行节点。未诊断到足以确认的构建环境根因，不把它写成源码编译错误。

## 源码所表达的构建方法

| 工程 | 依据 | 方法/限制 |
|---|---|---|
| SCAN2 | 根 README、ament CMake/package.xml | ROS2 Humble、C++17、colcon --symlink-install、Release；默认 CPU sensing |
| RM | 各包 ament CMake、根 shell 使用 install/setup.bash | colcon 工作区；含嵌套 LIO workspace、旧依赖名与外部库，尚不能宣称一次全量 build 已验证 |
| SCAN1 | AGENTS.md、catkin CMake | ROS1 Noetic/catkin_make；仅代码理解参考，不用于 ROS2 适配编译 |

未来经授权后可采用如下**隔离模板**，产物留在迁移工作区。它是待验证命令，不是本轮执行记录：

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base artifacts/scan/log build \
  --base-paths ../../SCAN-Planner-Ros2/src \
  --build-base artifacts/scan/build --install-base artifacts/scan/install \
  --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

RM 应先明确构建范围和 overlay，再用同样隔离方式选取 `hnurm_navigation`、`hnurm_bringup`、`hnurm_uart`、感知/配准及必要依赖；不要直接 source 当前 feature/odin 旧产物后声称测试了 main。

## 依赖登记

| 依赖 | 用途/位置 | 当前可用性 |
|---|---|---|
| ROS2 Humble、colcon | 两工程 ROS2 基础 | 路径/命令存在；未证明所有包齐全 |
| Nav2 + MPPI/Smac/STVL/ConstrainedSmoother 等 | RM launch 和 nav2_params.yaml | 本机有部分 Nav2 头文件/消息；部署版本与全部插件支持 UNKNOWN |
| Sophus、vikit、Livox SDK、PCL/OpenCV | FAST-LIVO2 CMake、driver | 存在源码/旧产物不等于版本/链接验证通过 |
| teaserpp、small_gicp、TBB/OpenMP | registration/Quatro CMake | 未做完整可用性检查 |
| Ceres/fmt、已删除相机包的残留声明 | hnurm_utils、hnurm_bringup manifest | 当前相机包已删除；先核对最小构建依赖，不恢复整套视觉作为 SCAN 前置 |
| Eigen、PCL、OpenCV、Armadillo | SCAN 算法与 simulator CMake | 未做完整可用性检查 |
| GLEW/GLFW/OpenGL/GLM | SCAN local_sensing 的 GPU 分支及相关构建 | GPU 不是默认；不可为文档任务安装 |
| ros_gz_*、gz_ros2_control | Go2 物理仿真入口 | 不等于确定性模拟器必须全部运行；可用性 UNKNOWN |

RM `vikit_common`、`quatro` 的 CMake 已使用 ament，但 package.xml 缺少与之匹配的明确 build_type 导出，之前 colcon discovery 将它们识别为 catkin；这是待核实构建元数据问题，未修改。`hnurm_bringup/package.xml` 还有本树未匹配的包名，FAST-LIVO2 有嵌套 install 库路径查找。`registration/params/default.yaml` PCD 路径含 `/home/rm/...`，属于运行输入配置问题，不用安装依赖解决。

发现缺依赖后先确认名称、版本、作用与当前环境，再向用户提交具体方案；本轮未 apt/pip/clone，也未修改任何 CMake/package.xml。
