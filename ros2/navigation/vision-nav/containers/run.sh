#!/usr/bin/env bash
# Run an isolated evaluation container. Run as an authorized Docker administrator.
set -euo pipefail
root="${WHEELTEC_SIM_ROOT:-$HOME/wheeltec_vision_sim}"
gpu="${WHEELTEC_SIM_GPU:-1}"
domain="${ROS_DOMAIN_ID:-87}"
[[ "$gpu" =~ ^[0-9]+$ ]] || { echo 'WHEELTEC_SIM_GPU must be an index' >&2; exit 2; }
[[ "$domain" =~ ^[0-9]+$ && "$domain" -gt 0 && "$domain" -lt 233 ]] || { echo 'Use an isolated nonzero domain <233' >&2; exit 2; }
mkdir -p "$root"
mkdir -p "$root/leases"
affinity=()
if [[ -n "${WHEELTEC_SIM_CPUSET:-}" ]]; then
  [[ "$WHEELTEC_SIM_CPUSET" =~ ^[0-9,-]+$ ]] || { echo 'Invalid CPU affinity list' >&2; exit 2; }
  affinity=(--cpuset-cpus "$WHEELTEC_SIM_CPUSET")
fi
exec docker run --rm --init --entrypoint /bin/bash --cpus "${WHEELTEC_SIM_CPUS:-8}" --memory "${WHEELTEC_SIM_MEMORY:-24g}" \
  "${affinity[@]}" \
  --shm-size 2g --device "nvidia.com/gpu=$gpu" --network bridge \
  -e ROS_DOMAIN_ID="$domain" -e ROS_LOCALHOST_ONLY=1 \
  -e IGN_PARTITION="wheeltec-vision-$domain" \
  -e WHEELTEC_SIM_LEASE_ROOT=/work/leases \
  -e OMP_NUM_THREADS="${WHEELTEC_SIM_OMP_THREADS:-2}" -e OPENBLAS_NUM_THREADS="${WHEELTEC_SIM_BLAS_THREADS:-1}" \
  -e OPENCV_FOR_THREADS_NUM="${WHEELTEC_SIM_OPENCV_THREADS:-2}" \
  -e HTTP_PROXY -e HTTPS_PROXY -e http_proxy -e https_proxy -e NO_PROXY -e no_proxy \
  -e NVIDIA_VISIBLE_DEVICES=void -e QT_QPA_PLATFORM=offscreen \
  -v "$root:/work" "${WHEELTEC_SIM_IMAGE:-wheeltec-vision-sim:humble-20261112}" /work/containers/entrypoint.sh "$@"
