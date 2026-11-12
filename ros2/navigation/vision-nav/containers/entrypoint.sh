#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [[ -d /work/ceres_install ]]; then
  export CMAKE_PREFIX_PATH="/work/ceres_install:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="/work/ceres_install/lib:${LD_LIBRARY_PATH:-}"
fi
if [[ -f /work/vins_ws/evidence/overlay.complete ]]; then
  source /work/vins_ws/install/local_setup.bash
fi
if [[ -f /work/sim_ws/evidence/overlay.complete ]]; then
  source /work/sim_ws/install/local_setup.bash
fi
exec "$@"
