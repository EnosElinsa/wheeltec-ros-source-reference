# wheeltec_vision_nav

WHEELTEC R680（`senior_akm`）的 D435C 视觉建图、定位与 Nav2 导航。默认运行 RGB-D＋轮速＋IMU 的 EKF；现有 VINS-Fusion 用于独立 A/B/C 离线评估，尚未晋级为在线驾驶里程计。

源码位于 `ros2/navigation/vision-nav/wheeltec_vision_nav`。车上构建到 `/home/wheeltec/vision_ws`，每个终端先加载 `source_overlay.bash`。此轮本地软件验证与待执行验收见 [IMPLEMENTATION.md](IMPLEMENTATION.md)，实车协议见 [ACCEPTANCE.md](ACCEPTANCE.md)。

## 数据与速度链路

| 链路 | 行为 |
| --- | --- |
| 相机 | 默认 848×480×30，投影逐帧交替；自然纹理左 IR 与相邻有效深度配对供 RGB-D VO |
| 配对 | 原始整数纳秒时间戳、真实差值、分类来源、丢弃原因发布到 `/vision/emitter_pair/provenance`；最大差值 50 ms，CameraInfo 必须与各自原图同戳 |
| 定位 | `/odom_vo` 经质量和协方差检查生成 `/vision/odom_vo`；新鲜轮速连续静止且陀螺仪波动合理时学习零偏；EKF 发布 `/odom` 与 `odom → base_footprint` |
| 地图 | RTAB-Map 发布地图与 `map → odom`；导航必须获得首次有效 `/localization_pose`，不能用 RViz 恒等 TF 作为就绪依据 |
| 避障 | 原始 `/camera/depth/image_rect_raw` 经有效深度门限与原始 CameraInfo 精确匹配，以原始时间戳转发到 `/vision/valid_depth/*` 后生成 `/vision/depth_cloud`，独立于 VO 配对；完整有效点云负责清除，`/vision/obstacles` 负责标记 |
| 速度 | 控制器与行为节点 → `/cmd_vel_nav` → 平滑器 → `/cmd_vel` → 看门狗 → `/cmd_vel_safe` → 底盘 |

相机位置默认沿用现有 URDF/CAD 的 `0.187 0.0175 0.132` m。`camera_xyz` 和 `camera_rpy`（弧度）支持完整外参；未给 RPY 时保留 URDF 姿态。这些值仍需实测核查，不能凭测试通过修改标定。

## 建图与导航

```bash
source /home/wheeltec/vision_ws/source_overlay.bash
ros2 launch wheeltec_vision_nav vision_mapping.launch.py \
  start_base:=true odom_source:=ekf use_wheel_guess:=true \
  database_path:=/home/wheeltec/.ros/vision_rtabmap_mapping.db
```

`start_base` 默认 false，只有显式 true 才启动执行器。`use_wheel_guess` 只启用轮速先验。`odom_source` 支持 `auto|ekf|vo_only|external`：auto 在启动底盘时选择 EKF，否则选择 VO 中继；external 要求外部唯一 `/odom` 与 TF 发布者。`hold_odom:=true` 是 vo_only 兼容入口，与显式 EKF/external 冲突时报错。

建图默认 `wipe_db:=false`。显式 true 时，取得本实例数据库锁并检查 SQLite 占用后，先生成包含 WAL 的一致备份，再清理指定数据库。备份路径进入运行配置。相机、串口和数据库冲突会明确报错，不使用广泛 `pkill`。这些资源锁用于遵守本协议的启动实例；不可替代对其他驱动入口的现场检查。

```bash
ros2 launch wheeltec_vision_nav vision_navigation.launch.py \
  start_base:=true odom_source:=ekf use_wheel_guess:=true autostart:=true \
  database_path:=/home/wheeltec/.ros/vision_rtabmap_mapping.db
```

启动等待数据、必要 TF 与定位就绪，默认 30 s 超时报错；Nav2 在就绪之后启动。已启动关键进程退出会关闭本实例。RViz 加载 `config/vision_runtime.rviz`，用 **2D Pose Estimate** 重新定位，用 **2D Goal Pose** 下发目标。

## 运行保护

`/vision/health/status` 提供 JSON，`/vision/health` 提供诊断，`/vision/ready` 为 transient-local Bool。状态为 INITIALIZING、READY、DEGRADED、STOPPED、RECOVERING。初始化持续零速；接收时间与保护定时器使用单调时钟，ROS 时间负责消息与 TF 对齐。重复旧时间戳不会刷新健康状态。

| 默认条件 | 取值 |
| --- | --- |
| 保护周期 | 50 ms |
| VO 消息/有效视觉结果 | 0.5 s |
| 轮速、IMU、融合里程计、有效深度/点云、odom TF | 0.3 s |
| 地图定位与 map TF / 指令超时 | 5 s / 0.5 s |
| 视觉单独失效续行 | 最长 1 s 或累计 0.10 m，最高 0.10 m/s，只前进，且轮速、IMU、融合、定位与避障有效 |
| 恢复 | 连续健康至少 1 s 且至少 10 个有效视觉样本 |
| 受控倒车 | 默认关闭；显式开启最高 0.08 m/s、连续最多 0.25 m 或 3.2 s |

严重故障输出零速、有限重试异步取消导航目标；恢复后等待新的已接受目标。取消未确认时禁止重新放行。建图故障恢复后需显式重新使能并发送新指令：

```bash
ros2 service call /vision/rearm std_srvs/srv/Trigger '{}'
```

只有在操作员确认后方净空的受控场地，才使用 `supervised_reverse:=true`。该模式统一规划器、MPPI、恢复行为与最终速度出口；视觉降级禁止倒车。前向相机不提供后方障碍物覆盖。

前向速度还受点云观测范围、车体轮廓、观测年龄和制动减速度约束。无返回不直接等于空闲，有效深度没有障碍物可以输出空障碍点云。当前制动减速度 0.3 m/s²、反应时间 0.15 s 与轮廓是待实测默认值。

底盘新增独立于串口传感器读取的稳态时钟指令超时线程。本包启动底盘时设为 0.3 s；原厂其他入口默认 0 表示兼容旧行为。看门狗退出、串口阻塞、整机断电或失联的实际停车能力必须分开验收。

## 参数和运行证据

[config/runtime.yaml](config/runtime.yaml) 统一 robot/perception/navigation/safety；用 `runtime_config:=/绝对路径/runtime.yaml` 加载副本。Nav2 默认读取包内 [visual_nav_baseline.yaml](config/visual_nav_baseline.yaml)，关键模板字段会校验。该基线尚待目标机验证。

首轮对照保持 10 Hz、0.3 m/s、MPPI `model_dt=0.1`、28 步、1000 采样；默认禁止全局穿越未知区域。车体 Ackermann 模型、轮廓和最小转弯半径在导航与保护中一致配置。

`monitor:=true` 默认开启，运行证据写入 `~/vision_runs/时间-进程号/`，可用 `output_dir` 修改。包含最终配置副本与哈希、软件/源码身份、地图文件身份、有效 ROS 参数快照、逐进程 CPU/RSS、温度、数据年龄、视觉处理时间、原始配对差值、地图修正和最终指令。地图运行中的文件哈希不代表冻结的 SQLite/WAL 快照。队列积压用数据年龄和处理耗时检查，未直接测量中间件队列占用量。

```bash
ros2 run wheeltec_vision_nav runtime_report /home/wheeltec/vision_runs/某次运行 \
  --output /home/wheeltec/vision_runs/某次运行/report-reviewed.json
```

`capture:=true` 额外保存预览到 `~/vision_captures`；图片编码和写入通过有界后台队列执行，丢弃/错误有记录。录包检查见 [BASELINE.md](BASELINE.md)。性能对比用同包回放逐项改变 848×480/640×480、特征数、点云降采样或 MPPI 采样数，并记录配置；只有数据年龄改善且定位/导航不退化才晋级。

## VINS 的复用

现有 [VINS.md](VINS.md) 继续提供 A（RGB-D＋EKF）、B（双目 VO）、C（双目 VIO）的同包回放、标定审计、构建证据和指标。在线保护复用 `vins_metrics.VisualHealth`，运行报告复用同一分布、视觉丢失率和独立真值指标。A 可传入本轮 `runtime_config`，B/C 始终使用已审计 VINS 标定。禁止把已经融合 IMU 的 VIO 与同一 IMU 作为独立高置信度观测再次融合；本轮没有提供 VINS 直接接管驾驶的自动切换。

## 开发与验证

```bash
python -m pip install -r requirements-dev.txt
python -m pytest -q -p no:cacheprovider
```

纯逻辑测试不需要 ROS。CI 增加 Python 3.10/3.12、安装入口、Humble 包构建与合成传感器集成测试；提交工作流不表示它已运行。目标环境执行方式和实车验收分别见 [ACCEPTANCE.md](ACCEPTANCE.md)。
