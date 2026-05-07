# UAV Autonomous Flight Stack

本仓库是一个基于 `ROS + PX4 + MAVROS` 的无人机自主飞行工程，包含深度相机/视觉惯导方案和激光雷达方案两套工作区。项目围绕复杂赛道中的自主感知、定位、轨迹规划和飞控执行展开，提供从传感器输入到飞控指令输出的完整任务链路。

## 项目结构

```text
SKBT
├── SKBT-Depth   # 深度相机 / 视觉惯导综合工程
└── SKBT-Lidar   # Livox 激光雷达工程
```

### SKBT-Depth

`SKBT-Depth` 是综合主工程，保留视觉、深度相机和部分雷达链路，主要包含：

- `src/realsense-ros`：RealSense 相机驱动与 T265 相关节点。
- `src/location/VINS-Fusion`：视觉惯性里程计。
- `src/location/faster-lio`：激光雷达惯性里程计。
- `src/location/odom_fusion`：统一里程计输出与高度融合。
- `src/detection/circle_detection`、`src/detection/EDcircles`：圆环、椭圆等视觉目标检测。
- `src/detection/barrier_detection`：基于深度图的障碍与窗口检测。
- `src/detection/pointcloud_detection`：点云滤波、聚类与通道中心提取。
- `src/planner/console`：任务状态机与关卡流程控制。
- `src/planner/minimumsnap`：Minimum Snap 轨迹生成。
- `src/planner/Apriltag`：AprilTag 识别与降落辅助。
- `src/control/px4ctrl`：PX4 Offboard 控制接口。

### SKBT-Lidar

`SKBT-Lidar` 是面向激光雷达方案的精简工程，主要包含：

- `src/location/faster-lio`：Livox / IMU 激光惯性定位。
- `src/location/odom_fusion`：统一里程计输出与高度融合。
- `src/detection/pointcloud_detection`：Livox 点云 ROI、滤波、聚类、窗口/通道中心估计。
- `src/control/px4ctrl`：PX4 Offboard 控制接口。
- `src/common/quadrotor_msgs`、`src/common/uav_utils`：公共消息与工具函数。
- `src/common/console`：控制状态消息依赖。

## 系统链路

整体链路可以概括为：

```text
传感器输入
-> 位姿估计 / 里程计融合
-> 目标检测 / 场景理解
-> 任务状态机
-> Minimum Snap 轨迹生成
-> PX4 外层控制
-> MAVROS / PX4 Offboard
```

典型数据流如下：

```text
Livox / RealSense / IMU / 测距
-> Faster-LIO / VINS-Fusion / T265
-> odom_fusion
-> /fusion/odom
-> console 状态机与轨迹规划
-> /console_position_cmd
-> px4ctrl
-> /mavros/setpoint_raw/attitude
-> PX4
```

## 核心模块

### 感知

感知模块针对圆环、窗口、隧道、森林通道等结构化场景设计，包含图像和点云两类处理链路：

- 图像侧使用圆/椭圆检测方法提取目标几何中心。
- 深度图侧根据障碍边界和孔洞结构估计可通过区域。
- 点云侧基于 PCL 完成 ROI 裁剪、滤波、欧式聚类和通道中心提取。

感知模块的输出不是直接控制指令，而是规划层使用的目标点、通道中心或几何约束。

### 定位与融合

定位来源根据平台配置不同而变化：

- 激光雷达方案：`Livox + IMU -> Faster-LIO -> /Odometry`
- 视觉惯导方案：`RealSense + IMU -> VINS-Fusion`
- T265 方案：`RealSense T265 -> VIO pose`

`odom_fusion` 负责将外部里程计和测距高度信息统一为 `/fusion/odom`。当前工程中加入了可配置的 `UKF` 融合路径，状态量为：

```text
[px, py, pz, vx, vy, vz]
```

滤波器使用常速度模型预测，通过外部 odom 位置和测距高度进行观测更新，输出连续的位姿与速度信息。

### 规划

规划层采用“任务状态机 + 航点生成 + Minimum Snap”的结构：

```text
当前关卡 / 当前阶段
-> 检测结果或预设目标
-> waypoints
-> Minimum Snap
-> p / v / a / yaw
```

状态机负责切换任务阶段，例如起飞、进入关卡、穿越圆环、通过窗口、调整航向和降落。每个阶段根据场景结构补充过渡点，避免直接从当前位置硬飞到目标点。

Minimum Snap 模块将离散航点转成平滑的时间参数化轨迹，使位置、速度、加速度和 jerk 在段间连续，便于后续控制器稳定跟踪。

### 控制

`px4ctrl` 负责把规划层输出的参考轨迹转换为 PX4 可执行的姿态和推力指令。控制链路包含：

- 位置指令接收。
- 自动起降状态管理。
- 推力模型估计。
- 姿态与推力指令发布。
- PX4 Offboard 模式接口。

当前控制层加入了可配置的滚动优化控制路径。控制器在预测时域内根据参考轨迹优化加速度序列，并在推力、姿态角和加速度约束下执行第一步控制，再转换为姿态和推力下发给 PX4。

## 编译

两个子工程都是独立的 catkin 工作区。根据使用的平台进入对应目录编译。

### 编译 SKBT-Depth

```bash
cd SKBT-Depth
catkin_make
source devel/setup.bash
```

### 编译 SKBT-Lidar

```bash
cd SKBT-Lidar
catkin_make
source devel/setup.bash
```

如果自定义消息尚未生成，可以先单独编译相关依赖包，再整体编译：

```bash
catkin_make --only-pkg-with-deps quadrotor_msgs
catkin_make
```

## 运行入口

具体启动顺序取决于传感器平台和飞控配置。一般顺序为：

1. 启动传感器驱动和定位节点。
2. 启动 `odom_fusion`，发布 `/fusion/odom`。
3. 启动 `px4ctrl`，连接 MAVROS / PX4。
4. 启动检测节点和 `console` 状态机。

常用入口示例：

```bash
roslaunch odom_fusion odom_fusion.launch
roslaunch px4ctrl run_ctrl.launch
```

实机运行前需要确认：

- PX4 参数与机体一致。
- MAVROS 已连接飞控。
- `/fusion/odom` 频率和坐标系正确。
- 遥控器模式、Offboard 模式和自动起降逻辑已验证。
- 推力映射和最大姿态角等控制参数经过实机调试。

## 配置说明

主要配置文件包括：

- `src/control/px4ctrl/config/ctrl_param_fpv.yaml`：质量、重力、控制增益、推力映射、NMPC 参数。
- `src/location/odom_fusion/launch/odom_fusion.launch`：里程计输入 topic、UKF 参数。
- `src/detection/pointcloud_detection/config/Cloth_Para.yaml`：点云滤波和布料仿真参数。
- `Px4_Paras/`：PX4 参数备份。

不同机体、传感器安装位置和飞控版本对应的参数可能不同，实机部署前应逐项核对。

## 依赖

工程主要依赖：

- ROS
- PX4 / MAVROS
- Eigen
- OpenCV
- PCL
- serial
- RealSense ROS
- Livox ROS Driver
- Ceres Solver

不同子模块的依赖不同。只使用激光雷达链路时，可以优先编译 `SKBT-Lidar`；需要视觉惯导、深度相机或 AprilTag 链路时，使用 `SKBT-Depth`。

## 注意事项

- 本仓库包含两个历史工程，部分第三方包和驱动源码保留在工程内，便于复现实机环境。
- `SKBT-Depth` 更完整，但依赖更多；`SKBT-Lidar` 更轻，适合只验证雷达链路。
- 实机飞行前必须先完成仿真、离线 bag 回放和低速悬停测试。
- 飞控参数、坐标系方向、推力模型和定位频率会直接影响飞行安全。

## 技术关键词

`ROS`、`PX4`、`MAVROS`、`C++`、`PCL`、`OpenCV`、`Eigen`、`RealSense`、`VINS-Fusion`、`Faster-LIO`、`Livox`、`UKF`、`Minimum Snap`、`NMPC`、`AprilTag`、`ED-Circles`、点云滤波、欧式聚类、轨迹规划、Offboard 控制。
