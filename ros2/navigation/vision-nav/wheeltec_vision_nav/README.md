# WHEELTEC isolated visual navigation

This repository copy lives at `src/wheeltec_vision_nav/`. Sync it to
`/home/wheeltec/vision_ws/src/wheeltec_vision_nav/` on the robot. Operator
handbook: [`docs/rough-guide.md`](../../docs/rough-guide.md) (应用 A).

This package adds a RealSense-only RGB-D mapping/localization path. It does not
start or modify Livox, `wheeltec_lidar`, `wheeltec_radar`, or the existing
WHEELTEC RTAB-Map launch files.

The D435C on this robot currently has missing Depth↔Color factory extrinsics,
so mapping uses **left infrared + raw depth** (shared optical center). Do not
feed color and unaligned depth into RTAB-Map. Color alignment stays disabled
until `rs-enumerate-devices -c` shows a non-zero Depth → Color extrinsics
matrix. Point cloud publishing is off during this bring-up. The driver runs
848×480 at 30 Hz with emitter on/off interleave; `emitter_pair` feeds
speckle-free IR plus projected depth to `rgbd_sync` at ~15 Hz.

Camera TF: the RealSense driver roots its tree at `camera_link`, so the URDF
`camera_joint` is rewritten at launch to the D435C position (`camera_xyz`,
default `0.187 0.0175 0.132` m in `base_footprint`, from the sensor-mount CAD).
Chain: `base_footprint → base_link → camera_link → camera_infra1_optical_frame`.
Override with `camera_xyz:="x y z"` after measuring on the car.

On the robot, load `/home/wheeltec/vision_ws/source_overlay.bash` rather than
only `vision_ws/install/setup.bash`.

## Mapping (stationary by default)

```bash
source /home/wheeltec/vision_ws/source_overlay.bash
ros2 launch wheeltec_vision_nav vision_mapping.launch.py \
  start_base:=false \
  wipe_db:=true \
  database_path:=/home/wheeltec/.ros/vision_rtabmap_mapping.db
```

`start_base:=false` does not start the chassis. Push the robot by hand.
`wipe_db:=true` (the default) passes `-d` and clears the database; set
`wipe_db:=false` to keep an existing map file.

Visual odometry publishes `/odom_vo` with `publish_tf` false. `vision_hold_odom_tf`
relays `/wheel/odom` onto `/odom` when the chassis is running, otherwise the
last valid visual pose, and broadcasts `odom` → `base_footprint` at 20 Hz.
The `hold_odom` launch argument is accepted and unused.

For slow mapping with a wheel/IMU guess (still not Nav2):

```bash
ros2 launch wheeltec_vision_nav vision_mapping.launch.py \
  start_base:=true \
  use_wheel_guess:=true \
  hold_odom:=false \
  database_path:=/home/wheeltec/.ros/vision_rtabmap_mapping.db
```

Motor enable stays a manual safety step (handbook section 1). Wheel TF is
the inverse pose `base_footprint` → `wheel_odom`. Visual odometry publishes
`/odom_vo`; with the chassis running it sets `guess_frame_id=wheel_odom` and
`Odom/ResetCountdown=1`, so a lost frame resets to the wheel prior instead of
freezing. `vision_hold_odom_tf` publishes `/odom` and `odom` → `base_footprint`,
so the robot frame has a single parent.

Use RViz to inspect `/map`, `/odom`, `/camera/infra1/image_rect_raw`, and
`/cloud_obstacles`. Load `config/vision_runtime.rviz` (also copied to
`~/.rviz2/vision_mapping.rviz` on the robot). The launch does not start RViz,
publish goals, or command velocities.

## Navigation (inactive by default)

```bash
source /home/wheeltec/vision_ws/source_overlay.bash
ros2 launch wheeltec_vision_nav vision_navigation.launch.py \
  database_path:=/home/wheeltec/maps/office.db \
  autostart:=false \
  start_base:=false
```

The navigation stack remains inactive until `autostart:=true`. Nav2 overrides
(`write_visual_nav_params`): the local costmap is voxel layer from
`/cloud_obstacles` + inflation only (no static layer, unknown space not
tracked); the global costmap is static `/map` + inflation; inflation 0.25 m is
matched by the MPPI `ObstaclesCritic`; the planner has `allow_unknown: true`;
MPPI `vx_max 0.3`, `vx_min -0.15`, `wz_max 1.0`. Reverse is allowed so Ackermann
can back out of a blocked heading. The behavior tree recovers by clearing
costmaps, a short `BackUp`, then waiting. It does not Spin.

When `start_base:=true`, the chassis publishes `/wheel/odom` with frames
`wheel_odom` / `base_footprint`. `vision_hold_odom_tf` writes the planar wheel
pose relative to the first sample onto `/odom`. Visual odometry still publishes
`/odom_vo` and uses `guess_frame_id=wheel_odom` as motion prior. TF `odom` →
`base_footprint` comes from `vision_hold_odom_tf`. Wheel TF is published as
`base_footprint` → `wheel_odom` (inverse pose) so `base_footprint` is not given
a second parent. `vo_watchdog` relays Nav2 `/cmd_vel` to `/cmd_vel_safe` for the
chassis. With the chassis running (`wheel_backup`), a VO loss alone does not
stop the car: `/odom` keeps flowing from wheel odometry and `/vision/vo_status`
reads `lost_wheel_backup`. The car stops and `navigate_to_pose` is cancelled
when `/wheel/odom` stops (0.3 s) or `map→odom` has not been refreshed for 5 s.
Without the chassis a VO loss stops the car immediately (`lost`).

RViz **2D Pose Estimate** publishes `/initialpose`, remapped to RTAB-Map for
relocalization after kidnapping. **2D Goal Pose** remains the Nav2 goal tool.

Before any real motion test, verify USB speed, IR/depth topics,
`odom -> base_footprint`, `base_footprint -> wheel_odom` (if the chassis is
on), `map -> odom`, costmap sources, emergency stop, and the selected robot
footprint.

VO / SLAM / grid parameters live in `config/rtabmap_visual.yaml`.

## Baseline bags (P1/P2 platform gate)

After the literature pass, record two separate bags without changing config:
vision+chassis, then M10P `/scan`. Do not mix those launches. Protocol:
[`BASELINE.md`](BASELINE.md).

```bash
ros2 launch wheeltec_vision_nav baseline_record.launch.py session:=vision_base \
  output:=/home/wheeltec/bags/r680_baseline_vision_base
ros2 run wheeltec_vision_nav check_baseline --session vision_base \
  --bag /home/wheeltec/bags/r680_baseline_vision_base
```

## Helpers

```bash
ros2 run wheeltec_vision_nav hold_odom_tf
ros2 run wheeltec_vision_nav vo_watchdog
ros2 run wheeltec_vision_nav emitter_pair
ros2 launch wheeltec_vision_nav capture_runtime.launch.py
```

`capture_runtime` writes infrared/depth/`/map` PNG previews and `summary.yaml` to
`~/vision_captures/`. Mapping and navigation also accept `capture:=true`.
