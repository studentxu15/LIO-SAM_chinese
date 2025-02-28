# LIO-SAM

**实时激光雷达惯性里程计软件包。我们强烈建议用户仔细阅读本文档，并首先使用提供的数据集测试软件包。该方法演示的视频可以在 [YouTube](https://www.youtube.com/watch?v=A0H8CoORZJU) 观看.**
<p align='center'>
    <img src="./config/doc/device-hand-2.png" alt="drawing" width="200"/>
    <img src="./config/doc/device-hand.png" alt="drawing" width="200"/>
    <img src="./config/doc/device-jackal.png" alt="drawing" width="200"/>
    <img src="./config/doc/device-livox-horizon.png" alt="drawing" width="200"/>
</p>

## 清单

- [LIO-SAM](#lio-sam)
  - [清单](#清单)
  - [系统架构](#系统架构)
  - [依赖](#依赖)
  - [安装](#安装)
  - [使用 Docker 平台来创建、部署和管理应用程序容器](#使用-docker-平台来创建部署和管理应用程序容器)
  - [准备激光雷达数据](#准备激光雷达数据)
  - [准备IMU数据](#准备imu数据)
  - [示例数据集](#示例数据集)
  - [运行软件包](#运行软件包)
  - [其他说明](#其他说明)
  - [服务](#服务)
  - [问题](#问题)
  - [论文](#论文)
  - [待办事项](#待办事项)
  - [相关包](#相关包)
  - [致谢](#致谢)

## 系统架构

<p align='center'>
    <img src="./config/doc/system.png" alt="drawing" width="800"/>
</p>

我们设计了一个系统，该系统的目的是维护两个图（graphs），并且能够在运行时表现得比真实时间快10倍。
  - 在 mapOptimization.cpp 文件中，因子图用于优化基于激光雷达和 GPS 的里程计因子。这个因子图在整个测试过程中保持一致，以确保位置估计的准确性。
  - 在 imuPreintegration.cpp 文件中，这个因子图用于同时优化 IMU 和激光雷达的里程计因子，并估计 IMU 的偏差。该因子图会定期重置，并保证在IMU频率下进行实时里程计估计。

## 依赖

这是LIO-SAM的原始ROS1实现。有关ROS2实现，请参阅分支 `ros2`。

- [ROS](http://wiki.ros.org/ROS/Installation) (用Kinetic和Melodic进行测试。 参见 [#206](https://github.com/TixiaoShan/LIO-SAM/issues/206) 对于 Noetic)
  ```
  sudo apt-get install -y ros-kinetic-navigation
  sudo apt-get install -y ros-kinetic-robot-localization
  sudo apt-get install -y ros-kinetic-robot-state-publisher
  ```
- [gtsam](https://gtsam.org/get_started/) (乔治亚理工学院开发的一个开源库，主要用于图优化、平滑和地图创建)
  ```
  sudo add-apt-repository ppa:borglab/gtsam-release-4.0
  sudo apt install libgtsam-dev libgtsam-unstable-dev
  ```

## 安装

使用以下命令下载并编译包。

```
cd ~/catkin_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd ..
catkin_make
```

## 使用 Docker 平台来创建、部署和管理应用程序容器
构建图像 (基于 ROS1 Kinetic):

```bash
docker build -t liosam-kinetic-xenial .
```

获得图像后，按如下方式启动容器：

```bash
docker run --init -it -d \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  liosam-kinetic-xenial \
  bash
```


## 准备激光雷达数据

用户需要以正确的格式准备点云数据以进行去畸变，这主要是在“imageProjection.cpp”中完成的。这两个要求是：
  - **提供点时间戳**. LIO-SAM 使用 IMU 数据来进行点云去畸变。因此，扫描中每个点的相对时间需要是已知的。最新的 Velodyne ROS 驱动程序应直接输出此信息。在这里，我们假设点时间通道被称为“time”。点类型的定义位于“imageProjection.cpp”的顶部。“deskewPoint()”函数利用这个相对时间来获取该点相对于扫描开始时的变换。当激光雷达以 10Hz 的频率旋转时，点的时间戳应在 0 到 0.1 秒之间变化。如果您使用其他激光雷达传感器，可能需要更改这个时间通道的名称，并确保它表示的是扫描中的相对时间。
  - **提供点的ring编号**. LIO-SAM 使用这些信息将点正确地组织在矩阵中。环号指示该点所属的传感器通道。点类型的定义位于“imageProjection.cpp”的顶部。最新的 Velodyne ROS 驱动程序应直接输出此信息。同样，如果您使用其他激光雷达传感器，可能需要重新命名这些信息。请注意，目前该软件包仅支持机械式激光雷达。

## 准备IMU数据

  - **IMU要求**. 与原始的 LOAM 实现一样，LIO-SAM 仅支持 9 轴 IMU，这可以提供滚转、俯仰和偏航的估计。滚转和俯仰的估计主要用于在正确的姿态下初始化系统。偏航的估计则在使用 GPS 数据时用于在正确的航向下初始化系统。理论上，像 VINS-Mono 这样的初始化程序将使 LIO-SAM 能够与 6 轴 IMU 一起工作。(**New**: [liorf](https://github.com/YJZLuckyBoy/liorf) 已增加对 6 轴 IMU 的支持。) 系统的性能在很大程度上取决于 IMU 测量的质量。IMU 数据速率越高，系统的精度越好。我们使用 Microstrain 3DM-GX5-25，它以 500Hz 的速率输出数据。我们建议使用至少具有 200Hz 输出速率的 IMU。请注意，Ouster 激光雷达的内置 IMU 是 6 轴 IMU。

  - **IMU对准**. LIO-SAM 将 IMU 原始数据从 IMU 坐标系转换到激光雷达坐标系，该坐标系遵循 ROS REP-105 规范（x - 前方，y - 左侧，z - 向上）。为了使系统正常工作，需要在 "params.yaml" 文件中提供正确的外部变换。 **之所以有两个外部变换，是因为我的 IMU（Microstrain 3DM-GX5-25）的加速度和姿态具有不同的坐标系。根据你的 IMU 制造商，可能会有两个外部变换相同，也可能不同。**. 以我们的设置为例：
    - 我们需要将 x-z 加速度和陀螺仪的读数设置为负值，以在激光雷达坐标系中转换 IMU 数据，这在 "params.yaml" 中由 "extrinsicRot" 指定。
    - 姿态读数的变换可能会略有不同。IMU 的姿态测量 `q_wb` 通常表示将 IMU 坐标系中的点旋转到世界坐标系（例如 ENU）。但是，算法需要的是`q_wl`，即从激光雷达到世界的旋转。因此，我们需要从激光雷达到 IMU 的旋转 `q_bl`，满足 `q_wl = q_wb * q_bl`。为了方便，用户只需在 "params.yaml" 中提供 `q_lb` 作为 "extrinsicRPY"（如果加速度和姿态具有相同的坐标，则与 "extrinsicRot" 相同）。

  - **IMU调试**. 强烈建议用户取消注释 "imageProjection.cpp" 中 "imuHandler()" 的调试行，并测试转换后的 IMU 数据的输出。用户可以旋转传感器套件，以检查读数是否与传感器的运动相对应。有关校正后的 IMU 数据的 YouTube 视频可以在 [这里 (link to YouTube)](https://youtu.be/BOUK8LYQhHs) 找到.


<p align='center'>
    <img src="./config/doc/imu-transform.png" alt="drawing" width="800"/>
</p>

## 示例数据集

  * 下载一些示例数据集以测试该软件包的功能。下面的数据集已配置为使用默认设置运行：
    - **行走数据集:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]
    - **公园数据集:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]
    - **花园数据集:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

  * 下面的数据集需要配置参数。在这些数据集中，点云主题是 "points_raw"。IMU 主题是 "imu_correct"，它以 ROS REP105 标准提供 IMU 数据。由于这个数据集不需要 IMU 变换，因此需要更改以下配置以成功运行该数据集：
    - "config/params.yaml" 中的 "imuTopic" 参数需要设置为 "imu_correct"。
    - "config/params.yaml" 中的 "extrinsicRot" 和 "extrinsicRPY" 需要设置为单位矩阵。
      - **旋转数据集:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]
      - **校园数据集（大型）:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]
      - **校园数据集（小型）:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

  * Ouster (OS1-128) 数据集。如果您使用默认设置，则无需更改该数据集的外参。请按照以下 Ouster 说明配置软件包以与 Ouster 数据一起运行。数据集的视频可以在以下位置找到：[YouTube](https://youtu.be/O7fKgZQzkEo):
    - **屋顶数据集:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

  * Livox Horizon 数据集。请参阅以下备注部分以获取参数更改的说明。
    - **Livox Horizon:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

  * KITTI 数据集。外参可以在下面的 KITTI 说明部分找到。要使用其他 KITTI 原始数据生成更多的袋子（bag），您可以使用 "config/doc/kitti2bag" 中提供的 Python 脚本。
    - **2011_09_30_drive_0028:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

## 运行软件包

1. 运行启动文件:
```
roslaunch lio_sam run.launch
```

2. 播放现有的bag文件:
```
rosbag play your-bag.bag -r 3
```

## 其他说明

  - **回环闭合:** 此处的回环功能提供了一个概念验证的示例。它直接改编自 LeGO-LOAM 的回环闭合。有关更高级的回环闭合实现，请参考[ScanContext](https://github.com/irapkaist/SC-LeGO-LOAM). 在 "params.yaml" 中将 "loopClosureEnableFlag" 设置为 "true" 以测试回环闭合功能。在 Rviz 中，取消选中 "Map (cloud)" 并选中 "Map (global)"。这是因为可视化的地图——"Map (cloud)"——仅仅是在 Rviz 中的一堆点云，它们的位置在姿态修正后不会更新。这里的回环闭合功能仅仅是改编自 LeGO-LOAM，它是一种基于 ICP 的方法。由于 ICP 运行速度较慢，建议将播放速度设置为 "-r 1"。您可以尝试 Garden 数据集进行测试。

  - **使用 GPS:** 提供了公园数据集以测试带 GPS 数据的 LIO-SAM。该数据集由 [Yewei Huang](https://robustfieldautonomylab.github.io/people.html)采集。要启用 GPS 功能，请在 "params.yaml" 中将 "gpsTopic" 更改为 "odometry/gps"。在 Rviz 中，取消选中 "Map (cloud)" 并选中 "Map (global)"。同时选中 "Odom GPS"，以可视化 GPS 里程计。可以调整 "gpsCovThreshold" 以过滤不良的 GPS 读数。"poseCovThreshold" 可用于调整将 GPS 因子添加到图中的频率。例如，当您将 "poseCovThreshold" 设置为 1.0 时，您会注意到轨迹会不断被 GPS 修正。由于 iSAM 优化较为复杂，建议将播放速度设置为 "-r 1"。

  - **KITTI:** 由于 LIO-SAM 需要高频率的 IMU 才能正常工作，因此我们需要使用 KITTI 原始数据进行测试。一个尚未解决的问题是 IMU 的内参未知，这对 LIO-SAM 的精度有很大影响。下载提供的示例数据，并在 "params.yaml" 中进行以下更改：
    - extrinsicTrans: [-8.086759e-01, 3.195559e-01, -7.997231e-01]
    - extrinsicRot: [9.999976e-01, 7.553071e-04, -2.035826e-03, -7.854027e-04, 9.998898e-01, -1.482298e-02, 2.024406e-03, 1.482454e-02, 9.998881e-01]
    - extrinsicRPY: [9.999976e-01, 7.553071e-04, -2.035826e-03, -7.854027e-04, 9.998898e-01, -1.482298e-02, 2.024406e-03, 1.482454e-02, 9.998881e-01]
    - N_SCAN: 64
    - downsampleRate: 2 or 4
    - loopClosureEnableFlag: true or false

<p align='center'>
    <img src="./config/doc/kitti-map.png" alt="drawing" width="300"/>
</p>

  - **Ouster 雷达:** 为了使 LIO-SAM 能够与 Ouster 激光雷达一起工作，需要在硬件和软件层面进行一些准备。
    - 硬件:
      - 使用外部 IMU。LIO-SAM 不支持 Ouster 激光雷达内部的 6 轴 IMU。您需要将一个 9 轴 IMU 附加到激光雷达上并进行数据收集。
      - 配置驱动程序。在您的 Ouster 启动文件中将 "timestamp_mode" 更改为 "TIME_FROM_PTP_1588"，这样您可以获得点云的 ROS 格式时间戳。
    - 配置:
      - 在 "params.yaml" 中将 "sensor" 更改为 "ouster"。
      - 根据您的激光雷达在 "params.yaml" 中更改 "N_SCAN" 和 "Horizon_SCAN"，例如，N_SCAN=128，Horizon_SCAN=1024。
    - Gen 1 和 Gen 2 Ouster:
      不同代数的点坐标定义可能会有所不同。请参考 [Issue #94](https://github.com/TixiaoShan/LIO-SAM/issues/94) 进行调试。

  - **Livox Horizon 雷达:** 请注意，固态激光雷达尚未与 LIO-SAM 进行广泛测试。在这里也使用外部 IMU，而不是内部 IMU。对这种激光雷达的支持基于与机械激光雷达的代码库最小变更。需要使用定制的 [livox_ros_driver](https://github.com/TixiaoShan/livox_ros_driver) 来发布可以被 LIO-SAM 处理的点云格式。其他 SLAM 解决方案可能提供更好的实现。欢迎更多的研究和建议。请更改以下参数以使 LIO-SAM 能够与 Livox Horizon 激光雷达一起工作：
    - sensor: livox
    - N_SCAN: 6
    - Horizon_SCAN: 4000
    - edgeFeatureMinValidNum: 1
    - 使用 [livox_ros_driver](https://github.com/TixiaoShan/livox_ros_driver) 对数据进行记录。

## 服务
  - /lio_sam/save_map
    - 将地图保存为PCD文件.
      ``` bash
        rosservice call [service] [resolution] [destination]
      ```
      - Example:
      ``` bash
        $ rosservice call /lio_sam/save_map 0.2 "/Downloads/LOAM/"
      ```

## 问题

  - **锯齿形或抖动行为**: 如果您的激光雷达和IMU数据格式与LIO-SAM的要求一致，那么这个问题很可能是由于激光雷达和IMU数据的时间戳未同步造成的。

  - **上下跳动**: 如果您开始测试您的 bag 文件，而 `base_link` 立即开始上下跳动，那么很可能是您的 IMU 外参设置错误。例如，重力加速度的值为负。

  - **mapOptimization 崩溃**: 这通常是由于 GTSAM 引起的。请安装 README.md 中指定的 GTSAM 版本。更多类似的问题可以在[这里](https://github.com/TixiaoShan/LIO-SAM/issues)找到。

  - **GPS 里程计不可用**: 这通常是由于消息的 frame_id 和机器人 frame_id 之间缺少可用的变换引起的（例如：应该从 "imu_frame_id" 和 "gps_frame_id" 到 "base_link" 之间提供变换）。请阅读[这里](http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html)机器人定位文档。

## 论文

如果您使用了任何此代码，请感谢引用 [LIO-SAM (IROS-2020)](./config/doc/paper.pdf)。
```
@inproceedings{liosam2020shan,
  title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
  author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5135-5142},
  year={2020},
  organization={IEEE}
}
```

部分代码是根据[LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)进行改编。
```
@inproceedings{legoloam2018shan,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```

## 待办事项

  - [ ] [Bug within imuPreintegration](https://github.com/TixiaoShan/LIO-SAM/issues/104)

## 相关包

  - [liorf](https://github.com/YJZLuckyBoy/liorf) LIO-SAM with 6-axis IMU and more lidar support.
  - [Lidar-IMU calibration](https://github.com/chennuo0125-HIT/lidar_imu_calib)
  - [LIO-SAM with Scan Context](https://github.com/gisbi-kim/SC-LIO-SAM)
  - [LIO-SAM with 6-axis IMU](https://github.com/JokerJohn/LIO_SAM_6AXIS)

## 致谢

  - LIO-SAM is based on LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time).
