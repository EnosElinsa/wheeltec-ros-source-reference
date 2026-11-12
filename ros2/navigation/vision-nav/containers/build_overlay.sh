#!/usr/bin/env bash
set -euo pipefail
[[ "${ROS_DISTRO:-}" == humble ]] || { echo 'Run inside the Humble image' >&2; exit 2; }
root="${WHEELTEC_WORK_ROOT:-/work}"
export VINS_BUILD_JOBS="${VINS_BUILD_JOBS:-4}"
export MAKEFLAGS="-j$VINS_BUILD_JOBS -l$VINS_BUILD_JOBS"
bash "$root/containers/build_ceres.sh"
export CMAKE_PREFIX_PATH="$root/ceres_install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$root/ceres_install/lib:${LD_LIBRARY_PATH:-}"
export Ceres_DIR="$root/ceres_install/lib/cmake/Ceres"
if [[ ! -f "$root/vins_ws/evidence/overlay.complete" ]]; then
  if [[ -e "$root/vins_ws" ]]; then
    echo "Previous overlay build is incomplete: inspect $root/vins_ws/evidence/build.log before resuming its build stages" >&2
    exit 1
  fi
  bash "$root/source/wheeltec_vision_nav/tools/build_vins_overlay.sh" "$root/vins_ws"
  touch "$root/vins_ws/evidence/overlay.complete"
fi
set +u
source "$root/vins_ws/install/local_setup.bash"
set -u
mkdir -p "$root/sim_ws/src"
for package in wheeltec_tf_guard wheeltec_sim_actuator wheeltec_vision_benchmark wheeltec_vision_sim; do
  if [[ -d "$root/source/$package" && ! -e "$root/sim_ws/src/$package" ]]; then
    ln -s "$root/source/$package" "$root/sim_ws/src/$package"
  fi
done
cd "$root/sim_ws"
colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source install/local_setup.bash
set -u
mkdir -p "$root/results/environment"
cp /opt/wheeltec/evidence/packages.tsv "$root/results/environment/"
ros2 pkg list > "$root/results/environment/ros-packages.txt"
python3 --version > "$root/results/environment/python.txt"
ign gazebo --versions > "$root/results/environment/gazebo.txt"
colcon test --event-handlers console_direct+
colcon test-result --verbose
WHEELTEC_RUN_ROS_TESTS=1 ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest -c /dev/null -q "$root/source/wheeltec_vision_nav/tests/test_ros_runtime.py"
mkdir -p "$root/sim_ws/evidence"
touch "$root/sim_ws/evidence/overlay.complete"
