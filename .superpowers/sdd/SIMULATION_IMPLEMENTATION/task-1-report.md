# Task 1 implementation report — EuRoC benchmark toolchain

Date: 2026-11-12. Base: `8b184d4b`. Implementation scope is the new independent
`ros2/navigation/vision-nav/wheeltec_vision_benchmark` ament_python package plus
this report. No existing `wheeltec_vision_nav` files were edited. Remote builds,
downloads and dataset runs remain the controller's work and are not claimed as
passed by this report.

## Interfaces delivered

- `validate_euroc --dataset SEQUENCE_OR_MAV0 [--output NEW_DIRECTORY] [--mode stereo|vio]`.
  Strict CSV/YAML validation, complete image decode/rectification validation,
  dataset identity report and optional measured fixed-extrinsic VINS config.
- `ros2 launch wheeltec_vision_benchmark euroc.launch.py dataset:=... output:=NEW_DIR
  mode:=vio|stereo run_estimator:=true|false readiness_timeout_s:=90 drain_seconds:=5`.
  Starts only RTAB-Map stereo odometry (when enabled) and the EuRoC player.
- `euroc_player --ros-args -p dataset:=... -p output_dir:=NEW_DIR -p use_sim_time:=true`.
  Direct player self-check; estimator execution goes through the launch so the
  config exists before the estimator starts.
- `trajectory_evaluate --dataset ... --trajectory ... --output-dir NEW_DIR
  [--tolerance-ns 10000000] [--interval-s 1.0]`. CSV required columns:
  `timestamp_ns,px,py,pz,qx,qy,qz,qw`, positions/orientations of the IMU/body origin.

`setup.py`, `setup.cfg`, ament resource marker, `package.xml`, installed launch and
README are included. Package XML declares ROS message/TF/RTAB-Map, NumPy, OpenCV,
PyYAML and matplotlib runtime dependencies. ROS-independent testing dependencies
are in `requirements-test.txt`. The existing pinned VINS build tool is referenced
as the backend prerequisite; the hardware metadata audit entry point is not used.

## Input/timing/geometry contract

The loader reads `mav0/cam0`, `cam1`, `imu0`, and
`state_groundtruth_estimate0` CSV/YAML plus every named image. It rejects duplicate,
descending, fractional/scientific/negative timestamps; duplicate stereo streams
must be exactly equal in integer nanoseconds. It validates bounded image paths,
file existence, calibrated pinhole/radial-tangential models, positive focal
lengths, finite distortion, matching resolutions, proper SE(3) extrinsics,
nonzero baseline, sensor rates, SI IMU column labels/order/noise parameters,
finite measurements, normalized GT quaternions, temporal IMU bracketing and GT
overlap. `validate_euroc` also decodes every image and checks its resolution.

OpenCV `stereoRectify` and `initUndistortRectifyMap` produce actual rectification
maps. Images use mono8. Published rectified CameraInfo uses rectified K/P, zero
distortion and identity R because the published optical frame is itself
rectified. Horizontal positive-x right-camera baseline is checked for VINS.

Transforms follow `T_parent_child`. The benchmark body is the physical IMU frame:
`base_footprint` has full 6DoF and is not a planar chassis reference. Camera static
geometry is `inv(T_BI) @ T_BC @ T_C_Crect`, with `R_C_Crect=R_rect.T`. GT is
`T_world_S @ inv(T_BS_gt) @ T_BI`. The raw/rectified camera TF chain and identity
`base_footprint -> benchmark_imu` are published statically. Generated OpenCV VINS
camera calibration has rectified pinhole intrinsics, zero distortion, fixed
extrinsics, `estimate_extrinsic=0`, `estimate_td=0`, and `td=0`; measured EuRoC
IMU noise parameters are retained.

All sensor header stamps are generated using integer `divmod(ns, 1e9)` from CSV
timestamps. The player merges raw IMU/stereo events, sends equal-time IMU first,
and paces 1x CSV elapsed time from a monotonic wall clock. It never replaces the
dataset time grid with nominal 20/200 Hz timers. `/clock` advances at about
200 Hz. Scheduling overload is exposed in CSV dispatch lateness and rectification
cost rather than hidden by re-stamping.

## ROS execution and evidence

Topics are `/benchmark/{left,right}/image_rect`, matching
`/benchmark/{left,right}/camera_info`, `/benchmark/imu`, `/benchmark/odom` and full
`/benchmark/odom_info`. GT is never published. RTAB-Map parameters include
`Odom/Strategy=9`, `frame_id=base_footprint`, `odom_frame_id=benchmark_odom`,
`publish_tf=false`, `Reg/Force3DoF=false`, exact stereo sync, explicit reliable
sensor QoS and matching IMU QoS. Startup waits for all four stereo subscriptions,
the IMU subscription in VIO, and the estimator's odom-info publisher. Initial
clock/static TF precede sensor data, followed by a discovery settling period.
The dataset ends naturally; configurable drain permits queued outputs, then
CSVs are flushed/closed and scoring/plot export completes before launch shutdown.

Each new result directory contains:

- `manifest.json`: every CSV/YAML/image SHA-256 and size, a deterministic combined
  dataset identity, generated config hashes, Python/OpenCV/NumPy versions,
  source-file hashes and exact estimator parameters/remaps.
- `config/{vins,left,right}.yaml` and new solver output directory.
- `trajectory.csv`: original odometry timestamp, full pose, frame IDs, receive
  monotonic time, data age, raw twist and both raw covariance matrices.
- `visual_status.csv`: stamp, reception/data age, loss, feature/match/inlier counts,
  estimator processing times.
- `playback.csv`: dispatch stamp/kind/time, schedule lateness and rectification time.
- `runtime.json`: actual sample counts, runtime failures, invalid output reasons,
  visual losses, timing/uncertainty interpretation and completion marker.
- `metrics.json` and shareable `trajectory.png`, including no-output/failed runs.

Results/config directories are created with `exist_ok=False`; CSVs/metric JSON
use exclusive creation. This prevents reusing a previous result accidentally.
No physical driver, navigation, EKF, wheel prior, dynamic odometry TF or reference
pose publisher is started.

## Honest uncertainty and reference provenance

The raw IMU has no measured orientation estimate: quaternion remains unavailable
and orientation covariance begins with -1. Angular/linear covariance stays zero,
meaning unknown under the ROS IMU convention; VINS receives measured noise
densities/random walks through config. Output covariance is recorded verbatim.
The pinned RTAB-Map VINS backend assigns heuristic 0.0001/9999 covariances, not
calibrated uncertainty; manifest/README say so and scoring does not weight by it.

Self-review of the pinned backend found it emits invalid zero-quaternion odom
while initializing/lost. The independent launch sets
`publish_null_when_lost=false`; full OdomInfo still records those frames as lost,
and coverage includes missing estimates. Unexpected malformed frames/poses or
non-increasing output timestamps are retained raw, excluded from scoring, and
mark the runtime failed. Estimator runs with no output/failed scores return a
nonzero player exit status; player-only self-checks intentionally have no output.

The supplied reference is independently loaded for scoring and withheld from
VINS, but must not be described as independent motion-capture orientation. The
actual MH sensor YAML says its reference is a nonlinear least-squares batch
solution over Leica position and IMU measurements including time-offset
estimation; orientation is only observed by IMU. Manifest preserves the reference
sensor YAML verbatim (including these comments), plus its hash. README/metrics
explicitly characterize rotational RPE as agreement with that supplied reference.

## Scoring definition

The scorer performs deterministic nearest-residual, unique, monotonic time
association within a default 10 ms tolerance. No reference or estimate can be
reused. It rejects nonmonotonic/duplicate trajectories. At least three matched
poses are required. Rigid SE(3) alignment fixes scale at 1; translation ATE reports
RMSE, median and max. Time-RPE compares SE(3) deltas at default 1.0-second interval
within the same tolerance and reports translational/rotational RMSE and pair
count. It does not interpolate or fit scale. Position-rank degeneracy is reported;
when positions cannot identify rotation, the first associated orientations resolve
that otherwise unobservable alignment component.

Coverage's denominator is every input stereo frame, including initialization,
GT endpoint gaps and tracking losses; `gt_supported_frames` makes endpoint gaps
visible. Status is `ok`, `partial_output`, `no_output` or `failed`, not an
acceptance threshold or navigation-readiness claim.

## Local verification (fresh final run)

Environment: Windows, Python 3.14, OpenCV 5.0.0, NumPy 2.x, matplotlib 3.10.8.
OpenCV/matplotlib were made available locally for genuine geometry/plot tests.

1. `python -m pytest ros2/navigation/vision-nav/wheeltec_vision_benchmark/tests -q`
   — **16 passed**.
2. `python -m pytest ros2/navigation/vision-nav/wheeltec_vision_nav/tests
   ros2/navigation/vision-nav/wheeltec_vision_benchmark/tests -q`
   — **526 passed, 1 skipped**, 3.19 s. The skip is the existing opt-in real ROS
   integration module; no new failure was suppressed.
3. From the new package directory:
   `python -m compileall -q wheeltec_vision_benchmark launch` — exit 0.
4. From the new package directory: `python setup.py --name`
   — exit 0, `wheeltec_vision_benchmark`.

Tests cover integer precision above 2^53, timestamp/order/pair/units/file errors,
proper/invalid SE(3), distortion and rotated-camera epipolar geometry, rectified
camera baseline, actual OpenCV FileStorage compatibility, IMU/GT sensor-origin
transform composition, rotation/quaternion roundtrip, additive rigid alignment,
scale errors surviving alignment, unique noncrossing matches, fixed-time rotation
RPE, scheduling order/stamp split, manifest/readiness parameter contracts, fresh
directory refusal, CLI validation/scoring, no-output JSON and actual PNG export.

The core tests were run red before implementation; later self-review fixes have
focused regression tests (noncrossing association, null-output policy, provenance
and Force3DoF). No ROS stack is installed locally, so rclpy callbacks, Humble
launch/colcon installation, actual VINS startup/QoS/TF behavior, MH sequence
accuracy, end-of-stream drain adequacy and remote performance require controller
execution. Those results must be added to the overall run report from real
artifacts, not inferred from local tests.
