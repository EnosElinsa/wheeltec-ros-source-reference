#!/usr/bin/env bash
# The pinned ROS2 VINS port uses ceres::Manifold (requires Ceres >=2.1).
# Keep the system Jammy Ceres untouched; all experiment source uses this prefix.
set -euo pipefail
root="${WHEELTEC_WORK_ROOT:-/work}"
prefix="$root/ceres_install"
source_dir="$root/ceres_source"
expected=f68321e7de8929fbcdb95dd42877531e64f72f66
if [[ -f "$prefix/share/ceres-build.complete" ]]; then
  [[ "$(cat "$prefix/share/ceres-build.complete")" == "$expected" &&
     -f "$prefix/lib/cmake/Ceres/CeresConfig.cmake" &&
     -f "$prefix/lib/libceres.so.2.1.0" ]] || { echo 'Ceres prefix identity is incomplete or unexpected' >&2; exit 2; }
  [[ "$(git -C "$source_dir" rev-parse HEAD)" == "$expected" &&
     -z "$(git -C "$source_dir" status --porcelain --untracked-files=no)" ]] || { echo 'Ceres source changed after build' >&2; exit 2; }
  sha256sum --check "$prefix/share/ceres-artifacts.sha256"
  exit 0
fi
if [[ ! -d "$source_dir/.git" ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/ceres-solver/ceres-solver.git "$source_dir"
  git -C "$source_dir" checkout --detach "$expected"
fi
[[ "$(git -C "$source_dir" rev-parse HEAD)" == "$expected" &&
   -z "$(git -C "$source_dir" status --porcelain --untracked-files=no)" ]] || { echo 'Expected clean pinned Ceres source' >&2; exit 2; }
cmake -S "$source_dir" -B "$root/ceres_build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DCUDA=OFF
cmake --build "$root/ceres_build" --parallel "${VINS_BUILD_JOBS:-4}"
cmake --install "$root/ceres_build"
mkdir -p "$prefix/share"
sha256sum "$prefix/lib/libceres.so.2.1.0" "$prefix/lib/cmake/Ceres/CeresConfig.cmake" > "$prefix/share/ceres-artifacts.sha256"
git -C "$source_dir" rev-parse HEAD > "$prefix/share/ceres-build.complete"
