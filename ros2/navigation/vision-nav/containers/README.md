# Humble / Fortress evaluation container

This image isolates the existing Humble baseline from a native Jazzy host. It
contains dependencies; VINS and the local packages build into the mounted work
directory. No physical driver starts. Use an unused ROS domain and a selected
available GPU. The stock binary RTAB-Map does not include VINS support.

```bash
export WHEELTEC_SIM_ROOT="$HOME/wheeltec_vision_sim"
mkdir -p "$WHEELTEC_SIM_ROOT"/{source,data,results}
cd ros2/navigation/vision-nav/containers
cp -a . "$WHEELTEC_SIM_ROOT/containers"
for package in wheeltec_vision_nav wheeltec_tf_guard wheeltec_vision_benchmark wheeltec_vision_sim wheeltec_sim_actuator; do
  cp -a "../$package" "$WHEELTEC_SIM_ROOT/source/"
done
sudo docker build -t wheeltec-vision-sim:humble-20261112 .
# When the host requires a proxy, preserve only the required proxy variables:
# sudo --preserve-env=HTTP_PROXY,HTTPS_PROXY,http_proxy,https_proxy \
#   docker build --network host --build-arg HTTP_PROXY --build-arg HTTPS_PROXY \
#   --build-arg http_proxy --build-arg https_proxy -t wheeltec-vision-sim:humble-20261112 .
sudo --preserve-env=HTTP_PROXY,HTTPS_PROXY,http_proxy,https_proxy,NO_PROXY,no_proxy \
  env WHEELTEC_SIM_ROOT="$WHEELTEC_SIM_ROOT" WHEELTEC_SIM_GPU=1 \
  bash build.sh
```

Docker native CDI requires a valid generated NVIDIA CDI specification; validate
with `nvidia-ctk cdi list`. `run.sh` maps only the selected index and restricts
resources. `nvidia-smi` validates mapping, while Gazebo camera output validates
rendering. The shared host driver must remain compatible with the image; the
container does not install or replace a host driver.

The runtime requests two OpenMP/OpenCV threads and one OpenBLAS thread via
`WHEELTEC_SIM_OMP_THREADS`, `WHEELTEC_SIM_OPENCV_THREADS` and
`WHEELTEC_SIM_BLAS_THREADS`. These variables alone do not guarantee bounded pools:
the tested OpenCV 4.5.4 ignored its environment limit and created 128 workers
with host-wide affinity. An explicit `WHEELTEC_SIM_CPUSET=0-7` reduced its observed
pool to eight. Choose available host CPUs and set `WHEELTEC_SIM_CPUSET` alongside
the CPU quota; `--cpus` limits CPU time but does not restrict visible affinity.
Record affinity, thread overrides, measured sim/wall ratio and input ages; keep
safety deadlines enabled.

The pinned VINS ROS 2 port requires the Ceres 2.1 manifold API. `build_ceres.sh`
builds a pinned CPU-only shared library in the mounted experiment directory;
VINS and RTAB-Map use that prefix explicitly. Cached source and installed
artifact identities are checked before reuse. Host and system Ceres packages
are not replaced.

`build.sh` uses host networking so a loopback-bound host proxy can download fixed
source dependencies. Its overlay build also runs four real ROS middleware tests
using synthetic safety-node inputs on fixed domain 91 with localhost-only DDS;
reserve that domain while building. These tests start no hardware drivers or
Gazebo actuator. `run.sh` uses a separate
bridge network for actual experiments. All interacting experiment processes must
run in the same container, because ROS_LOCALHOST_ONLY is enabled.

Runtime launchers use the persistent `/work/leases` directory inside the shared
work-root bind mount. Database ownership therefore locks the same inode across
containers targeting the same canonical `/work` database path, independent of
DDS domain. Do not remove lease files or replace that directory while sessions
run; stale metadata is harmless because ownership is held by the OS process
lock. World ownership is scoped to `IGN_PARTITION`, so independent transport
partitions and databases can run concurrently. All containers sharing a database
must use the same work-root mount and canonical database path.

The ROS apt repository uses HTTP with the official GPG key and signed InRelease
verification, because the repository HTTPS endpoint failed certificate-name
validation in the tested environment. TLS verification is not disabled. Package
versions are recorded in `/opt/wheeltec/evidence/packages.tsv`, and the Ubuntu
base is pinned by digest. The apt repositories are rolling, so preserve the built
image digest and package evidence for exact experiment reproduction.

## EuRoC data

Prefer the [ETH source](https://ethz-asl.github.io/datasets/euroc-mav/). When it is
rate limited, `fetch_euroc.py` retrieves the original nested sequence ZIPs from a
fixed public mirror revision. It verifies CRC/size and all inner ZIP entries,
computes local SHA256, and records the mirror origin and authenticity limit.
It downloads only selected members with HTTP byte ranges. Interrupted chunks
may be resumed; complete output directories are never overwritten.

```bash
python3 fetch_euroc.py --destination "$WHEELTEC_SIM_ROOT/data" --sequence MH_01_easy
python3 fetch_euroc.py --destination "$WHEELTEC_SIM_ROOT/data" --sequence MH_03_medium
python3 fetch_euroc.py --destination "$WHEELTEC_SIM_ROOT/data" --sequence MH_05_difficult
```

Retain download records and results separately from Git; datasets, images, build
trees and full run logs do not belong in the source repository. Publication
should contain compact measured reports, manifest hashes and reproducible commands.

After each `run_euroc_suite.sh` has finished, combine its `suite.json` and raw
timing CSVs into a new report. This preserves failed or missing runs, raw-data
and artifact hashes, coverage, ATE/RPE, and P50/P95/P99 for received odometry
age, estimator processing and stereo dispatch lateness. Negative/nonfinite
timing values remain explicit counts; scoring a partial trajectory grants no
algorithm or driving acceptance.

```bash
python3 /work/containers/summarize_euroc.py \
  /work/results/euroc-mh01-001 /work/results/euroc-mh03-001 /work/results/euroc-mh05-001 \
  --output /work/results/euroc-summary-001.json
python3 /work/containers/summarize_sim.py /work/results/mapping-001
```

## Session orchestration

Run launch and mission in one container so discovery and teardown are shared:

```bash
sudo env WHEELTEC_SIM_ROOT="$WHEELTEC_SIM_ROOT" WHEELTEC_SIM_GPU=1 \
  WHEELTEC_SIM_CPUS=32 WHEELTEC_SIM_CPUSET=0-31 ROS_DOMAIN_ID=88 \
  bash run.sh python3 /work/containers/run_sim_trial.py \
  --output /work/results/mapping-001 --database /work/results/room.db \
  --mode mapping --estimator baseline --scenario all
# Only after successful mapping and clean database shutdown:
sudo env WHEELTEC_SIM_ROOT="$WHEELTEC_SIM_ROOT" WHEELTEC_SIM_GPU=1 \
  WHEELTEC_SIM_CPUS=32 WHEELTEC_SIM_CPUSET=0-31 ROS_DOMAIN_ID=88 \
  bash run.sh python3 /work/containers/run_sim_trial.py \
  --output /work/results/localization-001 --database /work/results/room.db \
  --mode localization --estimator baseline --scenario all
```

This runner stores launch/mission logs and exit codes and stops the process
groups before returning. `session.json` records unexpected launch exit and each
shutdown signal/escalation. Session success requires mission exit 0, a launch
alive until intentional teardown, launch exit 0 after SIGINT, and no TERM/KILL
escalation. Mission observations remain saved when teardown fails; such a session
must not authorize localization from a supposedly clean map. Repeats share one evolving pose; independent repeats
require a new simulator launch and a fresh output directory. Preserve failed
sessions. Resource quotas are experiment settings: the eight-CPU smoke ran below
realtime and correctly triggered input deadlines. Raise the CPU quota or reduce
measured workload without relaxing safety deadlines, then record the achieved
sim/wall ratio. The initial source review does not establish live navigation
acceptance; see the maintained experiment report for actual outcomes.

Choose a CPU set available on the host; the example assumes at least 32 CPUs.
The quota limits aggregate CPU use, while affinity also bounds the CPUs visible
to libraries which size their worker pools automatically. Concurrent experiments
need separate CPU sets, DDS domains and database files. Record both limits;
changing resources does not authorize changing protection deadlines.

Measured tutorial and results are maintained in
[wheeltec-ros-textbook](https://enoselinsa.github.io/wheeltec-ros-textbook/08-advanced-applications/vision-simulation-lab/)
and its [experiment report](https://enoselinsa.github.io/wheeltec-ros-textbook/08-advanced-applications/vision-simulation-results/).
The report retains failed runs, dataset coverage, scene differences and the
remaining real-vehicle and VIO acceptance work.

## Stationary load recording

```bash
sudo env WHEELTEC_SIM_ROOT="$WHEELTEC_SIM_ROOT" WHEELTEC_SIM_GPU=1 \
  WHEELTEC_SIM_CPUS=16 WHEELTEC_SIM_CPUSET=0-15 ROS_DOMAIN_ID=92 \
  bash run.sh python3 /work/containers/run_soak_trial.py \
  --output /work/results/stationary-load-001 --duration 1800 --estimator baseline
```

This creates a new mapping database and sends no motion commands. Schema 2
records callback freshness continuously, clock progress, source health snapshots,
stationary wheel observations, raw safe-command receipt times, violation events,
1 Hz resource snapshots and source/manifest hashes. A missed deadline remains a
failure after recovery. The session records startup/recorder errors and bounded
teardown. A complete stationary run does not measure moving Nav2 control load,
acquisition-to-actuator latency or real-vehicle performance. Legacy schema 1
records lack the new receipt coverage; they cannot be upgraded retrospectively.
Source states/ages are evaluated when each health snapshot arrives; extrapolating
an old snapshot cannot prove that the underlying input stopped between reports.
Continuous direct source coverage requires separate input subscriptions.
