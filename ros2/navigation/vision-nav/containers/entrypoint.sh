#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [[ -f /work/vins_ws/install/local_setup.bash ]]; then
  source /work/vins_ws/install/local_setup.bash
fi
if [[ -f /work/sim_ws/install/local_setup.bash ]]; then
  source /work/sim_ws/install/local_setup.bash
fi
exec "$@"
