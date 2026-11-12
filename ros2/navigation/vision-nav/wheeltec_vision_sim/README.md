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
Raw Gazebo depth remains `32FC1` metres on `/sim/raw/left/depth_image`.
The simulation adapter explicitly converts to `16UC1` millimetres for the shared
depth-validity gate, preserving its header; NaN/infinite/nonpositive/overflow
returns become invalid zero. This preserves production obstacle protection.

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

Fault acceptance also requires physical stopping: after the command allowance
(0.55 s normally; 1.6 s for permitted visual degraded fallback), wheel speed must
fall below 0.02 m/s and 0.05 rad/s after a 0.85 s minimum braking allowance, and
independent GT drift must remain within 0.04 m. Pause checks frozen simulation
time and the resumed 0.3–1.5 s physical-stop window. `actuator_lease` deliberately
drops command delivery inside the native actuator while `/cmd_vel_safe` remains
live, proving its own timeout independently of ROS zero publishing. The drop
input can only remove command delivery; it cannot grant motion or renew a lease.

Every active trial sample requires fresh wheel, GT and all five contact streams;
the intentional paused interval and first 0.3 s after resume permit silent
sensors. Callback collision events are latched so a short hit cannot disappear
between recorder ticks. Route acceptance requires action success and fresh GT
within 0.25 m / 0.25 rad of the requested goal transformed by a **single fixed
initial map/world alignment** established from synchronous stationary start
poses. Endpoints never recompute alignment. Native measured joint publication
is explicitly throttled to 50 Hz even on older Fortress versions whose stock
JointStatePublisher ignores an `update_rate` element.
The native plugin also samples Gazebo's exact simulation time at 100 Hz on
`/sim/clock`, which the one-way bridge remaps to ROS `/clock`. The native 1 kHz
Gazebo `/clock` is not bridged, avoiding a callback storm in every ROS TimeSource.
Clock values retain the acquired integer nanoseconds; they are neither retimed
to wall time nor rounded to the sampling period. A pause emits no repeated
stationary clock; a backwards reset is published immediately even while paused.
Safety watchdog and native command expiry remain wall-clock protections.
Fortress's stock contact system publishes only nonempty events, so collision
coverage uses native heartbeats from physics-populated `ContactSensorData` on
all five real collision entities. Every physics-step hit is retained until the
50 Hz heartbeat; empty messages represent observed no-contact physics data,
never missing sensors. Publication requires every source component to exist.
Contact names are recovered from real entity IDs when Fortress omits names.
Only the exact generated `floor::link::floor_collision` identity is exempt as
expected support; unknown names or obstacles containing "floor" still count.
The mission runner limits command publication to 20 Hz wall time. The production
watchdog's full geometry evaluation runs only on its 20 Hz steady timer, so a
command burst cannot multiply cloud transforms or starve sensor/TF callbacks.
Zero, invalid or blocked commands still publish zero immediately; nonzero input
requires the next complete safety evaluation within 50 ms. Fixed 3D cloud
rotation avoids a multithreaded BLAS matrix dispatch. Health includes actual
input-error sources/messages and wall duration per safety tick for profiling.

Run local core checks with `python -m pytest .../wheeltec_vision_sim/tests`.
Run native math/deadline checks with `colcon test --packages-select
wheeltec_sim_actuator` and inspect `colcon test-result --verbose`. These checks
do not establish live sensor rates, navigation accuracy or collision-free
completion: only separately saved Gazebo run artifacts can establish those.
