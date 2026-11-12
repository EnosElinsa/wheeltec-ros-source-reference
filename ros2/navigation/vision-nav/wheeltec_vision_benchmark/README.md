# EuRoC stereo VO/VIO benchmark

This independent ROS 2 Humble package reads EuRoC raw stereo and IMU data, rectifies
the images, runs RTAB-Map VINS strategy 9, and scores against independently loaded
ground truth. It starts no physical drivers or navigation nodes. The VINS-enabled
RTAB-Map overlay from the sibling `wheeltec_vision_nav/tools/build_vins_overlay.sh`
is required for estimation; the distro RTAB-Map binary alone is insufficient.

“Independent” here means the scorer loads its reference separately and never
feeds reference poses to the estimator. EuRoC MH reference poses are a nonlinear
batch solution over Leica position and IMU measurements with time-offset
estimation; orientation is observed only by the IMU. Rotational RPE is therefore
agreement with that supplied reference, not an independent motion-capture
orientation accuracy claim. The manifest preserves the reference sensor YAML
verbatim, including its provenance comments, as well as its SHA-256 identity.

## Install and validate

Build this package into the same sourced workspace/overlay as the VINS-enabled
RTAB-Map installation. ROS dependencies are declared in `package.xml`.

```bash
source /opt/ros/humble/setup.bash
source /path/to/vins_eval_ws/install/local_setup.bash
colcon build --packages-select wheeltec_vision_benchmark --symlink-install
source install/local_setup.bash
ros2 run wheeltec_vision_benchmark validate_euroc \
  --dataset /data/MH_01_easy --output /results/mh01-validation --mode vio
```

The sequence directory or its `mav0` directory is accepted. All four sensors
(`cam0`, `cam1`, `imu0`, `state_groundtruth_estimate0`) need `data.csv` and
`sensor.yaml`; image paths must remain inside the camera `data` directory.
The validator decodes every image and validates its resolution. Terminal output
contains sample/file counts and the aggregate SHA-256 identity; `--output` saves
the complete per-file CSV/YAML/image identities in `validation.json`. Without
`--output`, no result directory is created. Missing GT is an error: playback for
this benchmark must remain independently scoreable.

Only the EuRoC 20 Hz pinhole/radial-tangential stereo and 200 Hz IMU calibration
are accepted. CSV times must be strictly increasing nonnegative integer
nanoseconds; stereo pairs must match exactly. The six IMU columns must be in
EuRoC gyro x/y/z `rad s^-1`, then accelerometer x/y/z `m s^-2` order. No sample
time is rounded, re-stamped, shifted, resampled or replaced with a timer grid.

Raw IMU and camera sequences need not share endpoints: for example, MH05's first
image precedes its first IMU sample by 20 ms. The loader and dataset hashes retain
every original sample. Stereo VO plays every image. VIO plays only images whose
original timestamps lie within the inclusive first-to-last raw IMU interval;
it skips unsupported leading/trailing images without padding IMU or changing
timestamps. `playback_policy` in the manifest, validation and runtime JSON records
every excluded original index/time with `before_first_imu` or `after_last_imu`,
and the counts of expected/playable/excluded stereo frames. Concise validation
stdout reports these counts. The replay wall epoch still starts at the original
earliest sensor timestamp. Coverage always counts **all original stereo images**,
so these exclusions reduce coverage explicitly instead of removing the loss
from the denominator. This endpoint policy does not interpolate or fill internal
IMU gaps.

Generated VINS noise uses the explicit `pinned_vins_midpoint_step_variance_v1`
policy. EuRoC YAML values are continuous densities per sqrt(Hz), while the pinned
VINS `integration_base.h` uses their squared parameters with discrete midpoint
weights. Two independent measurement endpoints each have weight `dt/2`, so
`acc_n` and `gyr_n` are density times `sqrt(2 * rate_hz)`. Bias driving noise has
weight `dt`, so `acc_w` and `gyr_w` are random-walk density times `sqrt(rate_hz)`.
Then the leading per-step variance is density squared times `dt`. This is the
pinned backend's independent-endpoint approximation; it does not account for
correlations caused by endpoint reuse across steps or claim exact multi-step
covariance. It is not the upstream example's empirically tuned noise setting.
Config generation checks every raw IMU interval against calibrated 200 Hz within
256 ns, permitting EuRoC's stored timestamp quantization but rejecting missing
samples or a conflicting rate; it never repairs or resamples them.
`manifest.json` records raw densities/units, measured interval extrema, source pin,
formulas and derived parameters/units under `imu_noise_policy`; raw dataset
identity remains unchanged. This policy corrects the previous direct density
copy and requires new controlled VIO runs before any accuracy claim. Preserve
the earlier failed trajectories and compare coverage/error without scale fitting.

## Run

Use an unused ROS domain so the benchmark `/clock` and TF do not reach a live
robot. Every `output` path must be new. A complete dataset hash pass occurs
before playback and may take some time.

```bash
export ROS_DOMAIN_ID=87
# Sensor/clock/TF self-check without an estimator:
ros2 launch wheeltec_vision_benchmark euroc.launch.py \
  dataset:=/data/MH_01_easy output:=/results/mh01-player run_estimator:=false

# Stereo visual odometry:
ros2 launch wheeltec_vision_benchmark euroc.launch.py \
  dataset:=/data/MH_01_easy output:=/results/mh01-vo mode:=stereo

# Stereo visual-inertial odometry:
ros2 launch wheeltec_vision_benchmark euroc.launch.py \
  dataset:=/data/MH_01_easy output:=/results/mh01-vio mode:=vio
```

The launch waits up to `readiness_timeout_s:=90` for the estimator's four stereo
subscriptions, its IMU subscription in VIO, and its odometry-info publisher.
Static TF and an initial clock are supplied before the first sensor sample.
Reliable QoS works with the explicitly reliable RTAB-Map subscriptions; reliable
publishers also support best-effort inspection subscribers. The player sends
samples at 1x elapsed CSV time using a monotonic wall clock. Equal-time IMU is
sent before stereo. `/clock` advances at approximately 200 Hz. Processing cannot
be forced to keep up: actual scheduling lateness and received data age are
recorded so an overloaded run remains visible.

After the last raw sample, `drain_seconds:=5` allows pending estimator output to
arrive, then the player closes CSVs, scores and plots, and shuts down the launch.
Increase the drain explicitly if late results remain. A self-check produces an
expected `no_output` score because no estimator ran. Interrupted/failed replay
still saves results when the player was initialized successfully.

The player may also run directly for inspection:

```bash
ros2 run wheeltec_vision_benchmark euroc_player --ros-args \
  -p dataset:=/data/MH_01_easy -p output_dir:=/results/direct-player \
  -p use_sim_time:=true
```

## Topics, frames and uncertainty

| Topic | Type / meaning |
| --- | --- |
| `/benchmark/left/image_rect`, `/benchmark/right/image_rect` | `sensor_msgs/Image`, mono8, rectified, original stamps |
| `/benchmark/left/camera_info`, `/benchmark/right/camera_info` | Matching `sensor_msgs/CameraInfo`, rectified K/P, D=0 and R=I |
| `/benchmark/imu` | Raw `sensor_msgs/Imu`, original SI readings and stamps |
| `/benchmark/odom` | `nav_msgs/Odometry`, estimator output, no TF publication |
| `/benchmark/odom_info` | Full `rtabmap_msgs/OdomInfo`, including loss/inliers/processing time |
| `/clock`, `/tf_static` | Replay clock and measured static sensor geometry |

Here **`base_footprint` is the physical EuRoC IMU origin and orientation**, with
full 6DoF. It has no chassis/ground-plane meaning. `benchmark_imu` is an identity
child of it. The raw optical frames are `benchmark_cam0_raw` and
`benchmark_cam1_raw`; the rectified optical frames are `benchmark_left_rect` and
`benchmark_right_rect`. The estimator's world frame is `benchmark_odom`.

EuRoC YAML matrices are interpreted as `T_BS` (sensor S into body B). The
published camera extrinsic is `inv(T_BI) @ T_BC @ T_C_Crect`, with
`T_C_Crect.rotation = R_rect.T`. Independent GT is transformed by
`T_world_S @ inv(T_BS_gt) @ T_BI`. Thus a nonidentity GT sensor transform or IMU
mounting cannot silently move the scored origin. GT is never published.

The input has no measured orientation estimate: IMU quaternion stays unavailable
and `orientation_covariance[0]=-1`. Gyro/acceleration covariance stays zero,
which means *unknown* under the ROS IMU message convention; measured noise
densities/random walks are supplied separately in the VINS config. Output
pose/twist/covariance are captured verbatim. The pinned RTAB-Map VINS backend
uses heuristic covariance values (0.0001 / 9999), **not calibrated statistical
uncertainty**. They are not used to weight this benchmark or claimed as measured
precision. `publish_null_when_lost=false` prevents invalid zero-quaternion
odometry during initialization/loss; `OdomInfo` and coverage preserve loss evidence.

## Results and independent scoring

Each new run directory contains `manifest.json` (dataset/config/source hashes,
versions and node arguments), fixed `config/{vins,left,right}.yaml`,
`trajectory.csv`, `visual_status.csv`, `playback.csv`, `runtime.json`,
`metrics.json`, and `trajectory.png`. `trajectory.csv` keeps original timestamps,
full pose/twist/frame IDs and both raw covariance matrices. Data age is the
receive monotonic wall time mapped to the 1x replay epoch minus sensor time;
`OdomInfo.time_estimation` is the estimator processing time. These are different
measurements and include different queue/scheduling effects.

The standalone scorer accepts a CSV with required columns
`timestamp_ns,px,py,pz,qx,qy,qz,qw` in the **IMU/body origin**, using xyzw
quaternions, not EuRoC's GT wxyz order. It also accepts the player's CSV when
all rows contain valid estimator poses.

```bash
ros2 run wheeltec_vision_benchmark trajectory_evaluate \
  --dataset /data/MH_01_easy --trajectory /results/mh01-vio/trajectory.csv \
  --output-dir /results/mh01-vio-rescore --tolerance-ns 10000000 --interval-s 1.0
```

Scoring associates timestamps uniquely and monotonically within 10 ms by
default (nearest residual first; deterministic ties). A sample cannot be reused.
At least three GT matches are needed. Alignment is rigid SE(3), with **scale
fixed at 1**. Translation ATE is computed after alignment; time-RPE uses pose
pairs separated by 1.0 s (within the same tolerance), reporting translation and
rotation RMSE. No interpolation or scale optimization hides missing data or
scale drift. Collinear/constant positions do not identify all alignment axes;
the line direction remains constrained by positions and the first associated
orientations resolve only the free rotation about that line. Constant positions
use the full orientation fallback. Position rank is reported.

Coverage is the fraction of *all input stereo timestamps* with a unique valid
estimate and associated GT, including initialization and tracking loss in the
denominator. `gt_supported_frames` exposes GT endpoint gaps. Status distinguishes
`ok`, `partial_output`, `no_output` and `failed`. A numeric score alone is not a
tracking-success or navigation-safety assertion.

## Local tests

```bash
python -m pip install -r ros2/navigation/vision-nav/wheeltec_vision_benchmark/requirements-test.txt
python -m pytest ros2/navigation/vision-nav/wheeltec_vision_benchmark/tests -q
```

Synthetic fixtures exercise large nanosecond timestamps, duplicate/noninteger
times, exact stereo pairs, units/files/calibration failures, distorted epipolar
rectification with rotated mounts, GT sensor/IMU transform composition, OpenCV
config compatibility, unique monotonic association, scale-sensitive SE(3) ATE,
fixed-time rotational RPE, manifests, CLI scoring, no-output JSON and plot export.
These tests do not establish real dataset accuracy or ROS/backend performance;
those require the controller's recorded Ubuntu runs.
