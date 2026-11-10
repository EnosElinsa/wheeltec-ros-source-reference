# R680 VINS-Fusion 离线评估

本轮实现的是 A/B/C 离线验证工具。在线导航基线为 RGB-D＋EKF，VINS 尚未接入在线导航；尚未在 R680 上编译、标定或证明定位效果。先验证 B（双目 VO），IMU 审计通过后才运行 C（双目 VIO）。本文实车命令仍待在 Ubuntu 22.04 / ROS 2 Humble 上执行。

## “VINS 接管驾驶”具体指什么

VINS 估计车辆的位置、朝向和速度；Nav2 根据定位和障碍物生成速度指令，最终由看门狗和底盘执行。“接管驾驶”指把导航依赖的里程计改为经过验证的 VINS 输出，VINS 本身不规划路线或控制电机。

目前代码复用了 VINS 实验的健康判定、录包审计、回放和统计，但 `/odom_vins` 仅用于离线比较。它没有连接在线 `/odom` 或驾驶 TF，也没有一键切换开关。通过下表实验后，还需实现在线坐标/速度/协方差适配和融合输入调整，再做导航验收。

| 条件 | 要确认什么 | 当前状态 |
| --- | --- | --- |
| 标定 | 左右相机的成像参数和基线；使用 IMU 时，还需相机到 IMU 的实测位置/姿态、IMU 单位、轴向及噪声。错误外参会把转动误解释为平移 | 工具能检查格式、与 CameraInfo/TF 的一致性；本次尚未验证本车的实测标定 |
| 同步 | 左右图像对应同一次曝光，IMU 时间表示真实采样时刻，能与图像对齐；不能只把消息时间戳改成相同 | 参考底盘驱动在串口收包后打主机时间戳，不能证明采样时间；现有审计会拒绝这类输入进入 C 组 VIO |
| 实时性 | 在车载平台按原速持续处理，数据年龄不增长、丢失后能恢复，同时不挤占避障与控制资源 | 已有 CSV、P95/P99 与资源统计；目标机同包回放、30 分钟负载和路线验收未执行 |

例如 IMU 的时间落后图像 50 ms，转弯时融合的可能是两个不同时刻的运动；算法仍可能输出位姿，却有系统性误差。又例如车辆以 0.3 m/s 行驶、定位落后 0.2 s，忽略延迟就会少算约 6 cm 的运动。因此“有输出”“测试通过”都不能直接证明可用于导航。

本车相机设置为 30 Hz 投影交替，筛选后的自然纹理帧名义约 15 Hz，需要测量实际有效频率。这里没有断言 D435C 无法使用 VINS：B 组双目 VO 不使用 IMU，可以先评估；C 组双目 VIO 的额外门槛是可信 IMU 时序与标定。消息头同戳和 `stereo_ready` 都不等于硬件同步或导航准入。

具体顺序是：构建固定版本后端 → 原始数据审计 → B 组与 A 基线同包比较 → 解决 IMU 时序并完成实测标定后评估 C → 实现在线适配 → ROS/受控场地导航与故障验收。未完成前保留当前 RGB-D＋EKF 在线链路。

## 调研结论及实现选择

VINS 把图像特征和 IMU 测量放入同一优化问题，估计位姿、速度及惯性偏置。推荐双目路线：双目提供尺度，先用无 IMU 版本检查相机链路，再比较 IMU 的真实收益。VINS 不能保证在完全无纹理、长期遮挡或错误闭环时继续正确定位；纯惯性传播也不能当作视觉跟踪正常。

| 组别 | 本实现入口 | 输出与用途 |
| --- | --- | --- |
| A `baseline` | 当前 RGB-D VO、轮速先验、陀螺仪及速度 EKF | `/odom`；重建当前里程计基线 |
| B `stereo` | RTAB-Map 内嵌 VINS-Fusion，双目、不使用 IMU | `/odom_vins`，`/vins/odom_info` |
| C `vio` | 同一后端，双目及完整原始 IMU | 同上；需审计与实际 TF 一致 |

回放不启动相机、底盘、Nav2 或地图优化，不发布速度指令；只播放原始输入及 `/tf_static`，排除录制的动态 `/tf`、`/odom` 和 `/clock`。A 会在独立实验域内重建 wheel / EKF 动态 TF；B/C 设置 `publish_tf=false`，不发布驾驶坐标变换。必须使用与实车不同且空闲的非零 `ROS_DOMAIN_ID`；程序无法自动判断其他机器的域号。

主要依据：

- [VINS-Mono 官方设备要求](https://github.com/HKUST-Aerial-Robotics/VINS-Mono#5-run-with-your-device)：建议图像 >20 Hz、IMU >100 Hz、准确时间戳。当前投影交替下的自然纹理帧名义约 15 Hz，需要实测评估，不能据此保证效果。
- [VINS-Fusion 官方仓库](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)支持单目＋IMU、双目＋IMU及双目 VO；官方原版使用 ROS 1，许可为 GPLv3。
- [Fusion 初始化分支](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L454)：双目具有独立初始化路径；不能把单目尺度初始化限制全部套用到双目。平面运动仍不适合依赖在线外参估计。
- [RTAB-Map VINS 后端](https://github.com/introlab/rtabmap/blob/afb63bd3380ab42710f3c70cd456665abd641997/corelib/src/odometry/OdometryVINSFusion.cpp)要求双目输入，策略为 `Odom/Strategy=9`；配置路径为 `OdomVINSFusion/ConfigPath`。[构建选项](https://github.com/introlab/rtabmap/blob/afb63bd3380ab42710f3c70cd456665abd641997/CMakeLists.txt)默认关闭。
- 同一后端会用相机、IMU 的 TF 覆盖 YAML 的相机—IMU 外参，并使用简化协方差。本工具因此校验录包里的静态 TF，而非只检查 YAML 矩阵格式。
- [官方独立节点输出](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/utility/visualization.cpp#L124)的线速度位于世界系，部分协方差等字段未填写。本实现使用 RTAB-Map 封装，不提供直接重映射官方节点到 EKF 的捷径。

## 1. 构建独立 overlay

先准备 Humble、现有 WHEELTEC / RealSense / RTAB-Map 依赖，以及 git、curl、cmake、colcon、C++ 编译器、Eigen、OpenCV、Ceres、Boost 和 Python pytest / yaml / numpy / psutil。使用已有可工作的底座环境，脚本不会运行 sudo 或写入 `/opt`。MCAP 录包还需对应的 ROS 2 storage 插件。

在机器人或同架构评估机上，从本包源码目录执行；目标目录必须不存在：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash  # 改为本车已有 underlay
VINS_BUILD_JOBS=2 bash tools/build_vins_overlay.sh "$HOME/vins_eval_ws"
source "$HOME/vins_eval_ws/install/local_setup.bash"
```

脚本固定以下来源，保存补丁、版本、CMakeCache、链接检查和构建日志到 `vins_eval_ws/evidence/`：

| 组件 | 固定版本 |
| --- | --- |
| [zinuok/VINS-Fusion-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2) | `72023bc5d2abb14faa4c02a5c040d84f996b587a` |
| [RTAB-Map core](https://github.com/introlab/rtabmap) | `afb63bd3380ab42710f3c70cd456665abd641997` |
| [rtabmap_ros](https://github.com/introlab/rtabmap_ros) | `140156edc807549787c89c166adc8e69b54c56ed` |
| [matlabbe 集成补丁](https://gist.github.com/matlabbe/ebbb343cd744da9d6d6ded2e1557fd) | `1a3991c08eef41a5ca6a9a1919c78008d3cb6de5` |

补丁 SHA256 为 `644c7fd1c2b2b32ed1645ae689eb2ac586f444ecc3933ff556dc6ba9cd5f7a49`。已针对固定 VINS 源码检查补丁可应用；Humble / Ceres 的实际编译兼容性仍需上述构建验证。外部 VINS 为 GPLv3；本包的 Apache-2.0 声明不改变外部组件或组合分发的许可要求。

此外应用本包 `tools/rtabmap_vins_visual_health.patch`：上游内点统计会包含滑窗里的历史观测，补丁仅将当前图像仍在观测的已求解特征计为内点；当前图像尚未被求解器处理时不沿用旧位姿发布成功。补丁及 SHA256 保存在构建 evidence 中。这里只修正输出与诊断，不改变估计器调度；若 IMU 不及时导致积压，实验会记录跟踪丢失，不能视为正常视觉传播。该补丁同样尚待 ROS 编译和遮挡场景验证。

## 2. 录制同一组原始数据

在实车通常使用的 ROS 域内，使用新数据库路径启动现有采集链路，增加右红外：

```bash
ros2 launch wheeltec_vision_nav vision_mapping.launch.py \
  start_base:=true record_stereo:=true wipe_db:=true \
  database_path:=/tmp/vins_acquisition_01.db
```

另一个同域终端开始录包，Ctrl-C 正常结束：

```bash
ros2 launch wheeltec_vision_nav vins_record.launch.py output:="$HOME/vins_data/trial_01"
```

`vins_record` 仅录制已运行的话题，不启动驱动。保存原始左右红外、两侧 CameraInfo、深度、三路 metadata、完整 `/imu/data_raw`、轮速及静态 TF；额外录制当前里程计和动态 TF 供检查，但不在实验中播放它们。输出目录必须是新目录。

另有相机专用 `vins_camera.launch.py`，适合单独检查相机；它不提供机器人模型、底盘 IMU 或 base TF，不能与上述相机入口同时运行，也不能仅靠相机录包完成 A/C。现有 mapping/navigation 未指定 `record_stereo` 时仍关闭右红外。

左右图像和各自 CameraInfo 必须具有相同的原始整数纳秒时间戳，禁止用相邻帧或重打时间戳凑对。`vins_stereo` 优先使用同戳投影 metadata；metadata 不足时沿用深度填充率及红外点阵分类启发式，无法分类的帧不输出。`/vins/stereo/status` 会报告实际来源；启发式不能证明投影状态或硬件同步，使用前需检查预览与 metadata，记录结果。当前录包保持投影交替设置，B/C 只接收判为投影关闭的同一时刻左右图像。现有 RGB-D 相邻配对不参与双目同步。

覆盖静止（独立静止包至少数秒）、起步停车、直行、转弯、弱纹理、短时遮挡、回到起点的闭环路线，每类至少三次。保留原始 bag、标定测量方法、参数、代码和构建证据。不要把轮速当真值。

## 3. 先审计并生成 B 配置

下面的 Linux 路径是示例。所有 JSON、配置目录和实验输出均拒绝覆盖已有文件：

```bash
mkdir -p "$HOME/vins_results"
ros2 run wheeltec_vision_nav vins_audit \
  --bag "$HOME/vins_data/trial_01" \
  --output "$HOME/vins_results/trial_01_stereo_audit.json" \
  --imu-timing-provenance host_receive
ros2 run wheeltec_vision_nav vins_make_config \
  --audit "$HOME/vins_results/trial_01_stereo_audit.json" \
  --mode stereo --output-dir "$HOME/vins_results/config_B"
```

审计退出码 0 只表示 `stereo_ready=true`；1 表示双目审计未通过，2 表示读包、输入或依赖错误。必须查看 JSON 的 `checks` 和 `vio_ready`，不能拿退出码作为 VIO 许可。审计实际反序列化消息，不会退化为仅检查 bag 元数据。

B 检查原始图像 ≥10 Hz、所有左右图像精确配对、每幅图像具有同戳 CameraInfo、稳定帧名和尺寸、有限且有效的整流投影矩阵与正基线。为减少不确定性，当前准入要求全部图像配对；有丢帧的包会失败，应先解决采集问题。此处 10 Hz 是实验准入门槛，不是 VINS 性能保证，筛选后的有效自然纹理帧率另看实验日志。

配置生成器从审计得到的整流 P 矩阵生成两侧零畸变针孔模型，不照搬 D435i。默认固定外参和时间偏移估计，禁用在线补全标定。

## 4. 同包运行 A、B

停止采集，另开评估终端并 source 实验 overlay。确保 87 是空闲且与实车不同的域，依次执行：

```bash
export ROS_DOMAIN_ID=87
ros2 launch wheeltec_vision_nav vins_replay.launch.py \
  mode:=baseline bag:="$HOME/vins_data/trial_01" \
  output:="$HOME/vins_results/trial_01_A"
ros2 launch wheeltec_vision_nav vins_replay.launch.py \
  mode:=stereo bag:="$HOME/vins_data/trial_01" \
  config:="$HOME/vins_results/config_B/vins.yaml" \
  audit:="$HOME/vins_results/trial_01_stereo_audit.json" \
  output:="$HOME/vins_results/trial_01_B"
```

默认按原速播放，用 `/clock` 驱动仿真时间，包播完后关闭节点。`rate:=0.5` 可用于调试，但原速实时性能结论必须用 `rate:=1.0`。不要同时在该域启动其他实验。输出含 manifest、逐条 odometry / visual / frames / resources CSV；健康信息在 `/vins/health`。只有持续前进时间戳、未丢失且足够内点的视觉结果刷新健康状态，惯性或 EKF 持续输出不计为视觉恢复。

manifest 记录 bag 元数据和全部数据文件的流式 SHA256、审计、两侧相机配置、主配置、包源码、运行参数及可用的 overlay 构建证据哈希。审计后修改 bag 数据会阻止回放；大包计算哈希可能需要时间。它记录身份信息，不代替保存原始文件。配置里的 `solver_output` 是上游临时目录；每次实验的证据以独立输出目录中的 CSV / JSON 为准，同一配置不要并发使用。

## 5. C 的额外准入

参考底盘驱动在串口接收后使用主机 `now()`，因此当前应声明 `host_receive`，C 将被拒绝。默认不改 STM32 固件。实测确认 IMU 时钟机制前，不要将参数改成 `sensor` 来绕过检查。在线 td 也无法修复随机串口延迟。

只有确有可信传感器采样时间及外部标定证据时，准备 `measured_imu.yaml`：`imu: 1`、`estimate_extrinsic: 0`、两组实测 4×4 `body_T_cam0/1`（从各整流相机光学坐标变换到 IMU 坐标）、`acc_n/gyr_n` 噪声密度、`acc_w/gyr_w` 随机游走、`g_norm` 及 `td: 0`。加速度单位 m/s²、角速度 rad/s；测量轴向、重力方向、尺度和噪声需实物验证。矩阵可用嵌套 YAML 或 OpenCV matrix。不要使用单位阵或官方 D435i 外参替代测量。

固定版本的 ROS 封装仅把 IMU 送到当前图像时间附近，而 VINS 的非零 td 可能需要更晚的数据；直接启用会使旧位姿带上新图像的时间戳。因此本实验额外要求 `td=0, estimate_td=0`。若标定测得非零时差，必须先实现可信的输入时钟同步或修复上游数据调度并验证，当前入口拒绝运行；不能把测量值改为零来绕过。

```bash
ros2 run wheeltec_vision_nav vins_make_config \
  --audit "$HOME/vins_results/trial_01_stereo_audit.json" \
  --mode vio --calibration "$HOME/vins_data/measured_imu.yaml" \
  --output-dir "$HOME/vins_results/config_C"
ros2 run wheeltec_vision_nav vins_audit \
  --bag "$HOME/vins_data/static_01" --stationary \
  --imu-timing-provenance sensor \
  --calibration "$HOME/vins_results/config_C/vins.yaml" \
  --output "$HOME/vins_results/static_01_C_audit.json"
ros2 run wheeltec_vision_nav vins_audit \
  --bag "$HOME/vins_data/trial_01" \
  --imu-timing-provenance sensor \
  --calibration "$HOME/vins_results/config_C/vins.yaml" \
  --stationary-audit "$HOME/vins_results/static_01_C_audit.json" \
  --output "$HOME/vins_results/trial_01_C_audit.json"
ros2 launch wheeltec_vision_nav vins_replay.launch.py \
  mode:=vio bag:="$HOME/vins_data/trial_01" \
  config:="$HOME/vins_results/config_C/vins.yaml" \
  audit:="$HOME/vins_results/trial_01_C_audit.json" \
  output:="$HOME/vins_results/trial_01_C"
```

C 采用保守实验门槛：IMU 严格 >100 Hz、最大间隔 ≤50 ms、p95 间隔 ≤20 ms、所有三轴数据有限、时间戳严格递增；相机时间＋td 必须位于 IMU 覆盖区间内（端点容忍 50 ms）。静止段需 ≥1 s、加速度模长与标准重力及配置 g 相差 ≤20%、加速度轴标准差的模 ≤0.5 m/s²、角速度均值模 ≤0.1 rad/s、标准差模 ≤0.05 rad/s。静止审计和运动审计须绑定同一配置 SHA256。它们是本实验的准入标准，不能替代同步标定，也不是 Fusion 的通用硬件规范。

回放启动前还读取 bag 的 `/tf_static`，确认 base、IMU 和两侧光学帧连通、无多父节点或环，两组实际相机—IMU 变换与标定逐元素相差 ≤1e-4，双目相对变换与整流基线一致。若录的是旧 CAD 位姿，与实测外参不符，必须更新正确的采集模型并重新录包，不可依靠 YAML 覆盖后端。B/C 都不进入现有 EKF。

## 6. 统计与下一阶段

```bash
ros2 run wheeltec_vision_nav vins_metrics \
  --run "$HOME/vins_results/trial_01_B" \
  --output "$HOME/vins_results/trial_01_B_metrics.json" --closed-loop
```

仅在物理路线确实回到起点时加 `--closed-loop`。报告包括初始化、有效视觉帧比例、丢失后恢复时长、未恢复尾段、轨迹长度、最大相邻位姿步长及端点位移。最大步长需要结合速度、帧率人工判定异常，不是预设合格阈值；一次 run 的 initialized 也不是多次试验成功率。

有独立同步真值时，加 `--reference /path/truth.csv --reference-source mocap`。CSV 列为 `stamp_ns,x,y,yaw`，位置必须换算到同一 base 坐标原点、角度为弧度；工具以 20 ms 最近邻唯一配对、初始 SE(2) 位姿对齐且不调尺度，给出平面 ATE、航向 RMSE、1 s 平移 RPE。这不是完整 6DoF 评测。没有真值时只比较闭环端点、重复性，地图一致性需后续独立建图试验验证，不能把端点误差称为 ATE。

CSV 保留每次输出的延迟和视觉处理时间，资源采样报告评估进程 RSS / CPU；CPU 百分比按多核进程口径可超过 100%。延迟是在回放 ROS 时钟下的输出时刻减图像/里程计戳，不能解释成硬件采样到控制的全链路延迟。保存 overlay 的 `evidence/` 与每次实验输出，固定速度与同一 bag 比较。

指标 JSON 汇总延迟和视觉处理时间的均值、p95、p99、最大值；资源统计仅跟踪本次 launch 的相关后代进程。无效位姿计数后从轨迹与真值配对中排除。`vins_metrics` 退出 0 表示有视觉初始化及里程计样本，1 表示未初始化（仍保存报告），2 表示输入或写入错误；这些退出码不表示精度或实时性能合格。

真实实验通过后再讨论接管导航：由唯一融合节点发布 `odom → base_footprint`、RTAB-Map 发布 `map → odom`，保留深度障碍物、地图和重定位。VIO 已使用同一 IMU 时需要重新设计 EKF 输入，避免陀螺仪信息重复融合；还要验证速度坐标系、从 IMU 到底盘的位姿转换及协方差。若时间质量不足，保留 B 结果，再评估具备同步时间戳的相机—IMU 硬件。本轮不提供自动切换到车辆控制的开关。


## 本轮导航保护对 VINS 的复用

在线状态机直接复用 `vins_metrics.VisualHealth` 的视觉新鲜度与内点判定；运行报告沿用同一分布、视觉丢失/恢复统计和独立真值关联。录包增加原始配对 provenance、EKF 质量和运行健康话题。

A 组现在可传入 `runtime_config:=/绝对路径/runtime.yaml`，复用本次运行的特征数、质量门槛和轮速先验相关性膨胀设置。manifest 记录该配置哈希。B/C 使用已审计的 VINS 配置；该参数不会重采样录制图像或覆盖 B/C 标定。比较不同相机分辨率需分别录制同一协议的数据。

```bash
export ROS_DOMAIN_ID=87  # 确认该域空闲且与实车隔离
ros2 launch wheeltec_vision_nav vins_replay.launch.py \
  mode:=baseline bag:="$HOME/vins_data/trial_01" \
  output:="$HOME/vins_results/A_profile_01" \
  runtime_config:="$(ros2 pkg prefix --share wheeltec_vision_nav)/config/runtime.yaml"
```

标定、原始左右图像/CameraInfo 同戳、IMU 时间质量和目标平台实时性通过之前，保留 RGB-D＋EKF 在线基线。VIO 自带 IMU 时不得再把同一 IMU 当作独立高置信度输入叠加。见 [ACCEPTANCE.md](ACCEPTANCE.md) 和 [IMPLEMENTATION.md](IMPLEMENTATION.md)。
