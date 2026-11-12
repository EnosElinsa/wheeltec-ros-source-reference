#!/usr/bin/env bash
# Build network may need the host loopback proxy. No simulation/actuator starts.
set -euo pipefail
root="${WHEELTEC_SIM_ROOT:-$HOME/wheeltec_vision_sim}"
[[ -f "$root/containers/build_overlay.sh" && -d "$root/source/wheeltec_vision_nav" ]] || {
  echo 'Copy containers/ and source packages to WHEELTEC_SIM_ROOT first' >&2; exit 2;
}
exec docker run --rm --init --entrypoint /bin/bash --cpus "${WHEELTEC_SIM_CPUS:-8}" --memory "${WHEELTEC_SIM_MEMORY:-24g}" \
  --shm-size 2g --network host -e ROS_DOMAIN_ID=87 -e ROS_LOCALHOST_ONLY=1 \
  -e HTTP_PROXY -e HTTPS_PROXY -e http_proxy -e https_proxy -e NO_PROXY -e no_proxy \
  -e VINS_BUILD_JOBS="${VINS_BUILD_JOBS:-4}" \
  -v "$root:/work" "${WHEELTEC_SIM_IMAGE:-wheeltec-vision-sim:humble-20261112}" \
  /work/containers/entrypoint.sh bash /work/containers/build_overlay.sh
