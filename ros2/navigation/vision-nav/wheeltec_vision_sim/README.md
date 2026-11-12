# Fortress Ackermann simulation

This is a separate Humble/Fortress entry point. It launches no serial, chassis or
RealSense driver. Build `wheeltec_sim_actuator`, `wheeltec_vision_sim`,
`wheeltec_tf_guard` and `wheeltec_vision_nav` in the overlay; stereo/VIO also need
the RTAB-Map strategy-9 VINS overlay built by `build_vins_overlay.sh`.

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch wheeltec_vision_sim sim.launch.py \
  mode:=mapping estimator:=baseline database_path:=/results/room.db \
  output:=/results/mapping-001
```

Use a new `output` directory every launch. Existing databases are preserved;
choose a new database for an independent mapping experiment. Stop mapping
cleanly before starting localization with the saved database:

```bash
ros2 launch wheeltec_vision_sim sim.launch.py \
  mode:=localization estimator:=baseline database_path:=/results/room.db \
  output:=/results/localization-001
```

`estimator:=stereo` / `estimator:=vio` select the VINS backend. Generated configs,
SDF, TF URDF and a manifest are saved inside `output`. Override geometry, sensor
xyz/rpy, rates and noise using `geometry:=/absolute/path/geometry.yaml`; the
installed `config/geometry.yaml` lists all fields. Noise is synthetic, and
estimator covariance floors are conservative heuristics, not calibrated error.
The camera uses a generated pinhole model. CameraInfo retains its image stamp;
the right `P[3]` is `-fx*baseline`. No emitter/projector metadata is manufactured.

The model has four rolling joints and two front steering joints. The rear axle
floor point is `base_footprint`. All command and encoder equations use that
origin. Wheel odometry integrates **measured joint positions**, not Gazebo model
pose. `/sim/ground_truth` comes from Gazebo's independent world-pose odometry
publisher and is consumed only by experiment recording/obstacle placement.

The safe command chain is Nav2 controller/behaviors → `/cmd_vel_nav` → velocity
smoother → `/cmd_vel` → existing `vo_watchdog` → `/cmd_vel_safe` → one-way Gazebo
bridge → native actuator. The actuator has a 0.25 s `steady_clock` deadline and
sets joint velocities to zero while paused and before the first resumed physics
step if the lease expired. The ROS watchdog also retains its wall-clock timer,
goal cancellation and explicit new-goal latch. Mapping commands/rearm use the
existing `/cmd_vel` and `/vision/rearm` boundary. Default navigation is forward;
`supervised_reverse:=true` applies the shared bounded reverse speed/distance/time
policy to Nav2 and watchdog. Rear blind space still requires supervision.

TF owners: RTAB-Map owns `map→odom`; EKF owns `odom→base_footprint`; robot state
publisher owns the generated static sensor TF. Gazebo ground-truth TF is never
bridged. Nav2 lifecycle activation waits for `/vision/ready`, which includes
valid, fresh RTAB-Map localization covariance and `map→odom` plus unique TF
authority. Both mapping and localization hold advisory leases on the world and
database, and a critical process exit tears down the launched group. Every ROS
node uses simulation time, including nested costmap nodes; watchdog and launch
deadlines remain wall clock. VIO removes EKF `imu0` entirely, disables the
correlated wheel/gyro degraded fallback, and rejects VO when raw IMU is stale.

Run real scenarios after the corresponding launch becomes ready:

```bash
ros2 run wheeltec_vision_sim mission_runner run --mode mapping \
  --estimator baseline --scenario all --repeats 3 --seed 17 \
  --launch-manifest /results/mapping-001/manifest.json \
  --output /results/mapping-faults-001 --ros-args -p use_sim_time:=true
ros2 run wheeltec_vision_sim mission_runner run --mode localization \
  --estimator baseline --scenario all --repeats 3 --seed 17 \
  --launch-manifest /results/localization-001/manifest.json \
  --output /results/localization-faults-001 --ros-args -p use_sim_time:=true
```

Mapping includes straight/turn/stop; localization includes action-based route
arrival. Both exercise vision, IMU, depth, pause/resume, retained old-goal latch
and a real collision obstacle. Three trials are mandatory. Individual cases can
be selected with `--scenario`. The runner controls sensor delivery through
`/sim/fault/{vision,imu,depth}` SetBool services and pause through the native
world-control service. It never publishes `/cmd_vel_safe`. Fault recovery alone
cannot release the old goal; a subsequent new goal or explicit mapping rearm is
required. `samples.jsonl` records raw original sensor/GT stamps, safe commands,
joint velocities, contacts, health, process CPU/RSS and sim/wall ratio;
`results.json` contains observed assertions. Missing evidence, failed services
or nonzero commands fail a case. Exit 2 means at least one scenario failed.

Run local core checks with `python -m pytest .../wheeltec_vision_sim/tests`.
Run native math/deadline checks with `colcon test --packages-select
wheeltec_sim_actuator` and inspect `colcon test-result --verbose`. These checks
do not establish live sensor rates, navigation accuracy or collision-free
completion: only separately saved Gazebo run artifacts can establish those.
