# R680 基线数据包

文献闭环之后的第一步：在**不改车载配置、不进入危险区域**的前提下，确认时间同步、TF、RGB-D、IMU、轮速，以及（另开会话）M10P `/scan`。这不是融合算法，也不是 Nav2 恢复；只证明传感器能按本车现有入口被录下来，并给出通过/失败。

同一时间只开一个应用。不要把应用 A 与原厂激光建图/导航叠在一次录制里。

## 安全

人员站在车旁，急停可立即按下。第一次让轮子受控前检查电源、电机使能、电池电压与车周净空。本基线默认**不发 `/cmd_vel`**。

## 会话 V：视觉 + 底盘（IMU / 轮速）

建图启动会清空所给数据库。必须用一次性路径，不要指向 `office.db` 或正在用的图。

```bash
source /home/wheeltec/vision_ws/source_overlay.bash
date
lsusb -d 8086:0b07
mkdir -p /home/wheeltec/bags

ros2 launch wheeltec_vision_nav vision_mapping.launch.py \
  start_base:=true \
  use_wheel_guess:=true \
  hold_odom:=false \
  wipe_db:=true \
  database_path:=/tmp/r680_baseline.db
```

另开终端，车静止、无速度指令，录至少 60 秒：

```bash
source /home/wheeltec/vision_ws/source_overlay.bash
ros2 launch wheeltec_vision_nav baseline_record.launch.py \
  session:=vision_base \
  output:=/home/wheeltec/bags/r680_baseline_vision_base
```

录完后在原启动终端 Ctrl+C，确认退出。然后检查：

```bash
ros2 run wheeltec_vision_nav check_baseline --session vision_base \
  --bag /home/wheeltec/bags/r680_baseline_vision_base
```

车上也可先做 15 秒直播抽检：

```bash
ros2 run wheeltec_vision_nav check_baseline --session vision_base --live --seconds 15
```

通过条件（硬项）：红外/深度约 10–20 Hz，`/odom` ≥ 5 Hz，`/imu/data_raw` ≥ 10 Hz，`/wheel/odom` ≥ 5 Hz，时钟不是 1970，`/odom` 四元数合法。

警告项（不单独判失败）：手推有纹理墙面时 `/odom_info_lite` 的 `lost` 比例应低（默认门槛 10%）、内点中位数应大于 20、848×480 红外应 ≥ 10 Hz（掉下去则考虑改回 640×480）、`start_base:=true` 时 `/tf` 上应有 `odom`→`base_footprint`，以及作为逆变换的 `base_footprint`→`wheel_odom`（不要出现 `wheel_odom` 作为 `base_footprint` 的父系）。静止无纹理时允许跟丢，但应在有纹理墙面再补一段。

需要 VO 质量时：保持同一套话题，车**缓慢手推**前把 `start_base` 改回 `false` 另录一包 `session:=vision`。带底盘猜测的建图用 `start_base:=true` 或 `use_wheel_guess:=true`，此时轮速 TF 是 `base_footprint`→`wheel_odom`（逆），`odom`→`base_footprint` 与 `/odom` 由 EKF（`vision_ekf`）融合轮速、陀螺仪与视觉里程计后发布。

## 会话 L：M10P 扫描

等会话 V 的进程全部退出。不要同时开 `vision_mapping` / `vision_navigation`。

雷达驱动入口仍以车上现用命令为准（本手册应用 B 尚未冻结启动文件名）。只录扫描与 TF：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
# 此处只启动本车已经在用的 M10P 驱动，不要再叠加底盘总启动或视觉建图

source /home/wheeltec/vision_ws/source_overlay.bash
ros2 launch wheeltec_vision_nav baseline_record.launch.py \
  session:=lidar \
  output:=/home/wheeltec/bags/r680_baseline_lidar
```

```bash
ros2 run wheeltec_vision_nav check_baseline --session lidar \
  --bag /home/wheeltec/bags/r680_baseline_lidar
```

硬项：`/scan` 约 8–16 Hz，`frame_id=laser`。只开驱动、不开模型时，`base_footprint`→`laser` 可以没有（警告，不是失败）。

## 不要做的事

- 不要改参数、外参、URDF 或为了过检查而重标定。
- 不要损坏传感器或执行器。
- 不要把应用 A 与原厂激光 SLAM 录进同一个 bag。
- 不要用这次基线声称融合或完整性监测已经实现。
