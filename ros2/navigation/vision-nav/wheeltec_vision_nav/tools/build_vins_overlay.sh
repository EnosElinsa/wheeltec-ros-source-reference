#!/usr/bin/env bash
# Builds only a fresh evaluation workspace. No sudo, /opt writes or chassis startup.
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: bash build_vins_overlay.sh /absolute/new/vins_eval_ws" >&2
  exit 2
fi
workspace="$1"
[[ "$workspace" = /* && ! -e "$workspace" ]] || { echo 'Workspace must be new and absolute' >&2; exit 2; }
for command in git curl cmake colcon python3 sha256sum; do
  command -v "$command" >/dev/null || { echo "Missing dependency: $command" >&2; exit 2; }
done
[[ "${ROS_DISTRO:-}" = humble ]] || { echo 'Source ROS 2 Humble and the factory/vision underlay first' >&2; exit 2; }
package_source="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$workspace/src" "$workspace/evidence"
exec > >(tee "$workspace/evidence/build.log") 2>&1
vins_ref=72023bc5d2abb14faa4c02a5c040d84f996b587a
core_ref=afb63bd3380ab42710f3c70cd456665abd641997
ros_ref=140156edc807549787c89c166adc8e69b54c56ed
patch_ref=1a3991c08eef41a5ca6a9a1919c78008d3cb6de5
checkout() {
  git clone --filter=blob:none --no-checkout "$1" "$2"
  git -C "$2" checkout --detach "$3"
}
checkout https://github.com/zinuok/VINS-Fusion-ROS2.git "$workspace/src/vins-fusion" "$vins_ref"
checkout https://github.com/introlab/rtabmap.git "$workspace/src/rtabmap" "$core_ref"
checkout https://github.com/introlab/rtabmap_ros.git "$workspace/src/rtabmap_ros" "$ros_ref"
curl --fail --location --retry 2 \
  "https://gist.githubusercontent.com/matlabbe/ebbb343cd744da9d6d6d6ded2e1557fd/raw/$patch_ref" \
  --output "$workspace/evidence/rtabmap-vins.patch"
echo "644c7fd1c2b2b32ed1645ae689eb2ac586f444ecc3933ff556dc6ba9cd5f7a49  $workspace/evidence/rtabmap-vins.patch" | sha256sum --check
# This pinned port already fixes Duration constructors and disables GPU_MODE.
# Exclude only those three already-resolved file hunks from the older gist.
patch_exclusions=(--exclude=loop_fusion/src/utility/CameraPoseVisualization.cpp
                  --exclude=vins/src/featureTracker/feature_tracker.h
                  --exclude=vins/src/utility/visualization.cpp)
git -C "$workspace/src/vins-fusion" apply --check "${patch_exclusions[@]}" "$workspace/evidence/rtabmap-vins.patch"
git -C "$workspace/src/vins-fusion" apply "${patch_exclusions[@]}" "$workspace/evidence/rtabmap-vins.patch"
git -C "$workspace/src/rtabmap" apply --check "$package_source/tools/rtabmap_vins_visual_health.patch"
git -C "$workspace/src/rtabmap" apply "$package_source/tools/rtabmap_vins_visual_health.patch"
git -C "$workspace/src/rtabmap" apply --check "$package_source/tools/rtabmap_vins_extrinsics.patch"
git -C "$workspace/src/rtabmap" apply "$package_source/tools/rtabmap_vins_extrinsics.patch"
cp "$package_source/tools/rtabmap_vins_visual_health.patch" "$workspace/evidence/"
cp "$package_source/tools/rtabmap_vins_extrinsics.patch" "$workspace/evidence/"
sha256sum "$package_source/tools/rtabmap_vins_extrinsics.patch" > "$workspace/evidence/extrinsics-patch.sha256"
sha256sum "$package_source/tools/rtabmap_vins_visual_health.patch" > "$workspace/evidence/backend-patch.sha256"
cp -a "$package_source" "$workspace/src/wheeltec_vision_nav"
printf 'VINS %s\nRTABMap %s\nrtabmap_ros %s\npatch %s\n' "$vins_ref" "$core_ref" "$ros_ref" "$patch_ref" > "$workspace/evidence/versions.txt"
git -C "$workspace/src/vins-fusion" diff > "$workspace/evidence/vins-local.patch"
git -C "$workspace/src/rtabmap" diff > "$workspace/evidence/rtabmap-local.patch"
cd "$workspace"
jobs="${VINS_BUILD_JOBS:-2}"
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'VINS_BUILD_JOBS must be a positive integer' >&2; exit 2; }
export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
export MAKEFLAGS="-j$jobs -l$jobs"
ceres_args=()
if [[ -n "${Ceres_DIR:-}" ]]; then ceres_args+=("-DCeres_DIR=$Ceres_DIR"); fi
colcon build --merge-install --base-paths src/vins-fusion --packages-select camera_models vins \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DGPU_MODE=OFF -DBUILD_TESTING=OFF "${ceres_args[@]}"
# setup scripts sometimes inspect unset variables.
set +u
source "$workspace/install/local_setup.bash"
set -u
cmake -S src/rtabmap -B build/rtabmap-core -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$workspace/install" -DWITH_VINS_FUSION=ON -DWITH_CERES=ON \
  -DBUILD_APP=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_PERF_TESTS=OFF -DWITH_QT=ON -DBUILD_TESTING=OFF "${ceres_args[@]}"
cmake --build build/rtabmap-core --parallel "$jobs"
cmake --install build/rtabmap-core
grep -q '^WITH_VINS_FUSION:BOOL=ON$' build/rtabmap-core/CMakeCache.txt
cp build/rtabmap-core/CMakeCache.txt evidence/rtabmap-core-CMakeCache.txt
# Old Humble exports share a generic include root. GCC ignores an -I override
# when that same directory also occurs as -isystem. Unique source/generated
# include paths keep this pinned overlay ahead of installed RTAB-Map headers.
overlay_headers="-I$workspace/build/rtabmap_msgs/rosidl_generator_cpp"
for package in rtabmap_conversions rtabmap_sync rtabmap_odom rtabmap_util rtabmap_slam; do
  overlay_headers+=" -I$workspace/src/rtabmap_ros/$package/include"
done
colcon build --merge-install --base-paths src/rtabmap_ros src/wheeltec_vision_nav \
  --packages-select rtabmap_msgs rtabmap_conversions rtabmap_sync rtabmap_odom rtabmap_util rtabmap_slam wheeltec_vision_nav \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  "-DCMAKE_CXX_FLAGS=$overlay_headers"
set +u
source "$workspace/install/local_setup.bash"
set -u
ldd "$workspace/install/lib/rtabmap_odom/stereo_odometry" | tee evidence/stereo-odometry-ldd.txt
grep -q 'libvins_lib.so' evidence/stereo-odometry-ldd.txt
if grep -q 'not found' evidence/stereo-odometry-ldd.txt; then
  echo 'Unresolved runtime dependency; keep this workspace for diagnosis' >&2
  exit 1
fi
python3 -m pytest -c /dev/null -q "$workspace/src/wheeltec_vision_nav/tests" \
  --ignore="$workspace/src/wheeltec_vision_nav/tests/test_ros_runtime.py"
echo "Evaluation overlay built: $workspace/install/local_setup.bash"
echo 'This build is not evidence of sensor calibration, tracking accuracy or navigation readiness.'
