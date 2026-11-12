# 视觉导航仿真实施记录

日期：2026-11-12。需求：在远程 Ubuntu 上完整实施 EuRoC 回归、简化 Ackermann 仿真、VINS 导航对照，并把教程与验证结果发布到 wheeltec-ros-textbook。

## 约束

- 保留现有 Humble 实车基线；新增独立包和仿真入口，不启动串口或 RealSense。
- 远程使用隔离的 Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 容器，选定空闲 GPU，限制资源；不修改已有主机 ROS 和驱动。
- 地面真值只用于评分，轮速来自关节/轮子模型；合成传感器须标注来源。
- 仿真相机允许直接同步输入，不伪造 D435 投影 metadata。
- 所有驾驶指令经现有 /cmd_vel_safe；保留墙钟断流保护、故障取消目标与新目标恢复边界。
- EuRoC 保留整数纳秒时间戳、原始相机/IMU 标定与坐标；评分使用同步独立真值，禁止通过尺度拟合掩盖误差。
- 本地测试、ROS 集成、数据集结果、仿真导航、实车验收分别记录；未经运行的结果不得写成通过。
- 源码实现发布到 wheeltec-ros-source-reference；教材和实验记录发布到 wheeltec-ros-textbook。保留两仓库原工作区的已有修改。

## Task 1: EuRoC 基准工具

在 ros2/navigation/vision-nav/wheeltec_vision_benchmark 新建独立 ament_python 包。提供 validate_euroc、euroc_player、trajectory_evaluate 入口及 euroc.launch.py。

读取 EuRoC mav0/cam0、cam1、imu0、state_groundtruth_estimate0 的 CSV、sensor.yaml、图像。验证单调整数时间、精确双目配对、文件存在、内外参与 IMU 单位。通过 OpenCV stereoRectify/initUndistortRectifyMap 对原始畸变图像整流，发布匹配的 CameraInfo、相机/IMU 静态 TF 和 /clock，图像 20 Hz、IMU 200 Hz 依照实际 CSV 时间原速播放；不可重打传感器时间戳。base_footprint 在此基准表示 IMU/body，保留完整 6DoF，不使用车体平面约束。输出话题 /benchmark/left/image_rect、/benchmark/right/image_rect、对应 camera_info 和 /benchmark/imu。真值不发送给估计器。

从数据集生成固定外参的 VINS 双目 VO/VIO 配置（不使用已有硬件 metadata 审计入口）；独立 launch 调用 RTAB-Map VINS strategy 9，publish_tf=false，frame_id=base_footprint，输出 /benchmark/odom 及 /benchmark/odom_info。可靠启动顺序、QoS 兼容、数据流结束后留下可评分结果；支持不运行估计器的 validate/player 自检。运行输出到新的结果目录，记录配置/数据身份、原始轨迹 CSV、处理延迟/数据年龄与视觉状态，不覆盖已有结果。

独立 SE(3) 评分：时间唯一关联，刚体对齐但不估计尺度，平移 ATE 与固定时间间隔 RPE、覆盖率、失败/无输出标记，保存 JSON 和可分享轨迹图（matplotlib）。EuRoC GT body/IMU 坐标与估计器保持一致。使用 meaningful synthetic fixtures 测试时间、整流几何、变换、关联和评分；ROS 模块之外的核心可在本地 pytest。

## Task 2: 简化车辆与导航闭环

新建 wheeltec_vision_sim ament_python 包，提供参数化 Ackermann SDF/URDF、带足够纹理的室内 world、Fortress ROS bridge、sim.launch.py 与独立实验启动命令。车体/四轮/前轮转向，参数包括尺寸、轴距、轮距、轮径、转向限位、传感器外参和噪声。接收 /cmd_vel_safe，仿真执行器本身具有命令超时。提供同步 RGB-D/双目、CameraInfo、IMU、由关节运动获得的轮速和独立 ground truth；保留原时间证据并明确合成来源。不连接物理驱动。

复用原有 RGB-D+EKF、RTAB-Map、深度有效性/障碍/清除和 watchdog；通过专用仿真输入适配绕过 D435 投影配对。所有 ROS 节点使用仿真时间，墙钟保护不变，暂停/断流时零速。TF 所有权保持唯一。先 mapping 建图，再 localization 导航；导航必须得到 RTAB-Map 可信 map->odom 后才就绪。支持 baseline/stereo/vio 估计器选择，VINS 输出需坐标、速度、协方差有效性和新鲜度适配；VIO 模式不可重复把同一 IMU 当独立 EKF 输入。前向/受控倒车设置与现有安全配置一致。

提供可重复 mission/fault runner，记录启动/直行/转弯/停车、路线到达、碰撞、视觉/IMU/深度断流、暂停恢复、旧目标锁存以及资源和时间指标；固定种子、每场景至少三次。核心适配/运动学/故障逻辑加入 meaningful tests。

## Task 3: 容器与远程实测

编写固定 Ubuntu 镜像摘要的 Dockerfile、构建/运行脚本和依赖清单；构建 Humble/Fortress/Nav2/RTAB-Map/VINS overlay，受限 CPU/内存/共享内存和独立 ROS_DOMAIN_ID，只映射选定 GPU。下载 MH_01_easy 后校验压缩包与展开数据身份，执行 VO/VIO 并评分；补充中等和困难序列与三次重复。运行 Gazebo headless 渲染、传感器和导航/fault 测试；产出真实运行报告与日志身份。遇到兼容问题在源码修复并保留证据。

## Task 4: 教材发布

更新 wheeltec-ros-textbook 第 50 章与关联 VINS/操作章节，提供版本选择、远程启动、EuRoC、仿真建图导航、故障实验、结果解释、复现命令和限制。源码链接固定最终 commit。维护实验状态表，使用实际产物和运行结果；不公开机器地址或凭据。运行教材链接/资源检查、pytest 与 mkdocs --strict，发布并检查 GitHub Pages。

## Task 5: 审查和交付

对各独立模块做规格及代码审查，修复影响正确性的问题；运行原有逻辑测试、安装入口和 ROS 集成检查。提交并发布源码与教材，保持主分支一致，记录可复现命令和未通过项，不把软件通过等同实车通过。

## 实施结果

进行中。正式结果在完成实测后填写。
