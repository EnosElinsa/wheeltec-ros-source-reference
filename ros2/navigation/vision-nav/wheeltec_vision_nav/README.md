# wheeltec_vision_nav

Camera-based mapping, localization and Nav2 navigation for the WHEELTEC R680
(`senior_akm`) with a RealSense D435C. The package lives in
`ros2/navigation/vision-nav/wheeltec_vision_nav` of this repository. On the
robot it is built in `/home/wheeltec/vision_ws/src/wheeltec_vision_nav/`.
Load `/home/wheeltec/vision_ws/source_overlay.bash` in every terminal.

The package does not start or modify Livox, `wheeltec_lidar`, `wheeltec_radar`
or the factory RTAB-Map launch files.

## Pipeline

| Step | Implemented by | This package's part |
| --- | --- | --- |
| Left infrared + depth at 848×480, 30 Hz, projector toggled every frame | `realsense2_camera` | launch arguments, `config/realsense_d435c.yaml` |
| Keep speckle-free infrared, pair it with the adjacent projected depth | **`emitter_pair`** (this package) | classifies each frame as projector on/off from the infrared bright-dot density and the depth fill ratio, because this D435C's metadata carries no emitter state |
| One RGB-D message (`/rtabmap/rgbd_image`) | `rtabmap_sync/rgbd_sync` | remappings |
| Visual odometry (`/odom_vo`) | `rtabmap_odom/rgbd_odometry` | GFTT/BRIEF, wheel pose as motion guess |
| Gyro without bias (`/vision/imu`), VO velocity with realistic variance (`/vision/odom_vo`) | **`ekf_inputs`** (this package) | learns the gyro yaw-rate bias while the wheels stand still; lower bound on the VO twist variances |
| Fused odometry (`/odom`, `odom → base_footprint`) | `robot_localization/ekf_node` | `config/ekf_visual.yaml`: wheel vx/vy/vyaw, gyro yaw rate, VO velocity with 3σ rejection |
| Map, loop closure, localization (`/map`, `map → odom`) | `rtabmap_slam/rtabmap` | `config/rtabmap_visual.yaml`; odometry read from TF with fixed variances |
| Local obstacles (`/vision/obstacles`) | `rtabmap_util/point_cloud_xyz` + `obstacles_detection` | fed from the paired depth |
| Planning and control | Nav2 (Smac Hybrid-A*, MPPI) | `write_visual_nav_params`, Ackermann behavior tree |
| Stop on stale localization | **`vo_watchdog`** (this package) | relays `/cmd_vel` to `/cmd_vel_safe` |

Camera TF: the RealSense driver roots its tree at `camera_link`. The URDF
`camera_joint` is rewritten at launch to the D435C position (`camera_xyz`,
default `0.187 0.0175 0.132` m in `base_footprint`). Chain:
`base_footprint → base_link → camera_link → camera_infra1_optical_frame`.

The D435C reports zero Depth↔Color extrinsics, so the colour stream stays off
and left infrared is used as the RGB-D image.

## Mapping

```bash
ros2 launch wheeltec_vision_nav vision_mapping.launch.py \
  start_base:=true wipe_db:=true \
  database_path:=/home/wheeltec/.ros/vision_rtabmap_mapping.db
```

`start_base:=true` starts the chassis node (odometry on `/wheel/odom`, IMU on
`/imu/data_raw`), `wheel_odom_tf` and the EKF. Motors stay disabled until the
operator enables them. `wipe_db:=true` (default) clears the database.
With `start_base:=false` the chassis is not started, the robot is pushed by
hand and `hold_odom_tf` relays the visual pose to `/odom`.

After mapping, check the graph:

```bash
ros2 run wheeltec_vision_nav inspect_map_db /home/wheeltec/.ros/vision_rtabmap_mapping.db
```

It prints the loop closures that disagree with odometry, the heading
information of the odometry links and the largest jump of the map correction,
and exits non-zero when one of them looks wrong.

## Navigation

```bash
ros2 launch wheeltec_vision_nav vision_navigation.launch.py \
  start_base:=true autostart:=true \
  database_path:=/home/wheeltec/.ros/vision_rtabmap_mapping.db
```

RTAB-Map runs in localization mode on the saved database. Nav2 overrides
(`write_visual_nav_params`):

- local costmap: voxel layer from `/vision/obstacles` plus inflation, in `odom`;
- global costmap: static `/map` plus inflation; the planner may cross unknown cells;
- inflation 0.25 m, matched by the MPPI `ObstaclesCritic`;
- controller 10 Hz with MPPI `model_dt` 0.1 s × 28 steps, 1000 samples;
- `vx_max 0.3`, `vx_min -0.15`, `wz_max 1.0`, `PreferForwardCritic` weight 5,
  Smac `reverse_penalty` 3.0;
- behavior tree: clear costmaps, short `BackUp`, wait. No Spin.

`vo_watchdog` relays Nav2 `/cmd_vel` to `/cmd_vel_safe` and sends zero when
no command has arrived for 0.5 s. With the chassis running a VO loss alone
does not stop the car. During a VO loss the car stops and the goal is
cancelled when `/wheel/odom` stops for 0.3 s or RTAB-Map has not published a
`/localization_pose` for 5 s (`map → odom` on `/tf` is re-broadcast
continuously and cannot tell how old the correction is).

RViz: load `config/vision_runtime.rviz`. **2D Pose Estimate** publishes
`/initialpose` to RTAB-Map for relocalization; **2D Goal Pose** is the Nav2 goal.

## Captures and baseline

Mapping and navigation accept `capture:=true`. `capture_runtime` writes image
previews and `summary.yaml` (rates, TF parents including `/tf_static`, emitter
pairing status, EKF and controller parameters) to `~/vision_captures/`.

Baseline bag protocol: [`BASELINE.md`](BASELINE.md).

```bash
ros2 run wheeltec_vision_nav emitter_pair
ros2 run wheeltec_vision_nav vo_watchdog
ros2 run wheeltec_vision_nav hold_odom_tf
ros2 launch wheeltec_vision_nav capture_runtime.launch.py
```
