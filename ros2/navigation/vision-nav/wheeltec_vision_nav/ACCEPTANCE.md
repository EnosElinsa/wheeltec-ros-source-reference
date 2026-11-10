# 视觉导航验证与晋级协议

本协议把本地逻辑、ROS 集成、回放和实车结论分别记录。所有未执行项记为 `not_run`；配置文件或 CI 工作流存在不表示目标机已经通过。运行软件基线见 [IMPLEMENTATION.md](IMPLEMENTATION.md)。

## 1. 本地软件验证

在仓库 `wheeltec-ros-source-reference` 根目录运行：

```bash
python -m pip install -r requirements-dev.txt opencv-python-headless psutil
python -m pytest -q -p no:cacheprovider
python -m compileall -q ros2/navigation/vision-nav/wheeltec_vision_nav
```

测试覆盖启动无数据、传感器断流、旧时间戳、时间跳变、非法姿态/协方差、视觉限时限距降级、恢复滞回、旧指令清除、取消重试、受控倒车、配对原始差值、轮速断流零偏学习、锁/数据库备份及启动动作。独立 C++ 截止时间测试：

```bash
g++ -std=c++14 -Wall -Wextra -pedantic \
  -I ros2/robot/turn-on-wheeltec-robot/include \
  ros2/robot/turn-on-wheeltec-robot/test/test_command_deadline.cpp \
  -o /tmp/test-command-deadline
/tmp/test-command-deadline
```

该独立测试不等于完整 ROS 底盘编译或串口试验。完整底盘构建还需本车 `serial`、`wheeltec_robot_msg` 等 underlay 依赖。

## 2. ROS 2 Humble 集成

Ubuntu 22.04 上加载 Humble 与现有 WHEELTEC 依赖，在专用 workspace 构建本包。先运行不含执行器的合成传感器测试，使用与实车不同且空闲的非零 ROS 域：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
colcon build --packages-select wheeltec_tf_guard wheeltec_vision_nav
source install/setup.bash
ros2 pkg executables wheeltec_vision_nav
export ROS_DOMAIN_ID=93
export ROS_LOCALHOST_ONLY=1
export WHEELTEC_RUN_ROS_TESTS=1
python3 -m pytest -q /源码仓库/ros2/navigation/vision-nav/wheeltec_vision_nav/tests/test_ros_runtime.py
```

合成测试实际创建 ROS 消息、QoS、TF、指令出口和取消服务，验证就绪放行、点云断流零速及目标锁存。配套 `wheeltec_tf_guard` 使用 Humble C++ 接口读取真实 TF 发布者 ID，验证重复发布者、错误父坐标系和监测节点退出；运行记录测试验证 ROS 数据年龄及参数快照。该包必须与 Python 包一起构建，缺失或监测状态停发时看门狗不放行。还需在目标系统核验完整 RealSense/RTAB-Map/Nav2 插件、QoS 和状态发布语义：

- `/odom` 与 `odom → base_footprint` 唯一发布者，导航 `map → odom` 由 RTAB-Map 发布。
- `controller_server` 与 `behavior_server` 只输出 `/cmd_vel_nav`，平滑器输出 `/cmd_vel`，底盘只订阅 `/cmd_vel_safe`；Ackermann 旁路被关闭。
- 原始深度经有效性检查，以原始同戳的图像与 CameraInfo 转发到 `/vision/valid_depth/*` 再生成点云，避障不依赖 `/vision/paired/*`；合法无障碍点云可为空。
- `/localization_pose` 的实际发布周期、时间戳含义与 QoS 支持 5 s 门槛，旧 TF 重播不代替新定位。
- 任一关键节点退出、就绪超时、相机/串口/地图冲突都以明确原因结束本实例；其他实例不受误杀。
- 完整底盘节点在目标 underlay 编译，验证看门狗线程不依赖正常传感器包或 ROS 回调推进。

CI [vision-nav.yml](../../../../.github/workflows/vision-nav.yml) 提供 Python 与 Humble 自动检查入口；查看实际 run 日志后再填写通过状态。

## 3. 同包回放与故障注入

按 [BASELINE.md](BASELINE.md) 保存原始 bag；按 [VINS.md](VINS.md) 在隔离域执行 A/B/C。A 使用与实车相同的 runtime 配置。B/C 必须通过各自标定与时间审计。对同包的每次参数实验保存配置哈希、代码版本、原始数据身份、运行负载和报告。

故障表的每项至少重复三次，记录注入时刻、最后有效原始时间戳、状态变化、首个零速时刻、取消请求/结果与恢复后目标身份：

| 注入 | 期望 |
| --- | --- |
| 启动无数据/缺 TF/缺首次定位 | 持续零速，30 s 就绪超时；建图不要求地图定位 |
| VO 消息停止、视觉失败或只重发旧消息 | 0.5 s 判失效；仅在轮速/IMU/融合/避障/TF/导航定位有效时允许降级 |
| 轮速、IMU、融合、原始有效深度或点云停止 | 对应关键输入 0.3 s 后停车，不用其他重复消息续命 |
| 定位或 map TF 停止前进 | 各自最后有效输入 5 s 后停车，不能相互刷新 |
| NaN、Inf、非法四元数或非 PSD 使用协方差 | 拒绝样本，不更新健康证据；关键输入故障停车 |
| ROS 时间暂停、回退或跳跃 | 单调保护继续推进；回退/大跳变清除旧证据并锁存停车 |
| 取消服务迟到、失败、暂不可用 | 有限重试并记录结果，未确认取消不重新放行 |
| 恢复、旧目标仍在/旧指令持续输入 | 健康至少 1 s＋10 个视觉样本；旧指令清除、取消确认后等待新目标 |
| 配对失败/CameraInfo 迟到/分类冲突 | 不伪造可信配对；原始深度避障链独立；以原始差值评价同步 |
| 倒车关闭、降级中倒车或预算耗尽 | 零速、取消目标；开启时速度/距离/时间均受限 |

关键输入故障首个零速应不晚于“最后有效观测的配置超时＋两个保护周期（0.10 s）”。这是指令出口延迟指标，实际停止距离必须独立测量。VO 降级不得超出 1 s / 0.10 m / 0.10 m/s；受控倒车不得超出 0.08 m/s / 0.25 m / 3.2 s。记录测量采样误差和物理惯性，不能把软件零速当作车辆静止。

## 4. 实车三次重复与三十分钟运行

操作员在车旁可立即急停。先架空轮子或在受控低速场地验证速度出口、退出与失联停车，再进行路线场景。只在操作员确认后方净空的场地启用受控倒车。断主机或串口完全失联时，必须确认下位机/独立硬件的超时停车；未具备时不能完成实车准入。

生成 9 类场景×3 次重复的记录模板，工具本身不启动机器人：

```bash
ros2 run wheeltec_vision_nav acceptance_suite --init /home/wheeltec/vision_trials.json
ros2 run wheeltec_vision_nav acceptance_suite --report /home/wheeltec/vision_trials.json
```

场景包括静止、直线、转弯、闭环、弱纹理、遮挡、动态障碍、窄通道、受控倒车。每次填写真实运行目录、pass/fail/not_run、耗时、最小间距、人工干预、停车距离及失败原因。未知测量保持 null。将总记录保存为某次运行目录的 `trials.json` 后，运行报告会合并统计。故障注入表另附在验收记录中，不以路线成功代替故障测试。

每次检查地面、细桌腿、低矮障碍和动态物体移走后清除；覆盖相机遮挡/恢复、重复旧帧、低纹理和地图定位丢失。对障碍物静止/移走分别保存深度、完整点云、障碍点云和 costmap；不得把无深度区域当作已清空。

目标平台连续运行至少 30 分钟。`monitor:=true` 保存原始 CSV/JSONL、manifest 和参数。退出后生成报告：

```bash
ros2 run wheeltec_vision_nav runtime_report /home/wheeltec/vision_runs/实际目录 \
  --output /home/wheeltec/vision_runs/实际目录/report-reviewed.json
```

报告检查视觉有效帧率、丢失率、恢复时间、控制间隔超限比例、处理与消息年龄 P95/P99、逐进程 CPU/RSS、温度、内存斜率、地图修正平移/转角跳变。对长期日志检查积压与趋势，不凭平均延迟判断健康。控制间隔统计默认把超过 1.5 个 0.1 s 周期记为超限，并保留原始分布。

定位报告附回到起点的重复性和地图一致性；只有独立同步真值才使用 `--reference 真值.csv --reference-source 真值系统名称` 输出 ATE/RPE。轮速或 RTAB-Map 自身不是真值。导航报告比较成功率、人工干预、最小间距、耗时与失败原因，并与相同路线基线比较。

## 5. 配置晋级记录

每次只改变一个主要变量：848×480/640×480、特征数、点云降采样、MPPI 采样数或制动参数。记录原配置/新配置哈希、相同 bag/场景、三次结果、30 分钟负载和裁决理由。只有控制周期达标、数据年龄改善且定位/导航不退化才允许成为新默认。保持当前 10 Hz、0.3 m/s、2.8 s 预测时域作为首轮对照；实测轮廓、最小转弯半径和减速度必须写入配置和验收记录。
