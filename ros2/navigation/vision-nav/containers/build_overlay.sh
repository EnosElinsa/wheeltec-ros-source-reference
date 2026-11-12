#!/usr/bin/env bash
set -euo pipefail
[[ "${ROS_DISTRO:-}" == humble ]] || { echo 'Run inside the Humble image' >&2; exit 2; }
root="${WHEELTEC_WORK_ROOT:-/work}"
export VINS_BUILD_JOBS="${VINS_BUILD_JOBS:-4}"
if [[ ! -f "$root/vins_ws/install/local_setup.bash" ]]; then
  bash "$root/source/wheeltec_vision_nav/tools/build_vins_overlay.sh" "$root/vins_ws"
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
