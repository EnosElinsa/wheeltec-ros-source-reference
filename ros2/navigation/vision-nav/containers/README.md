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
  bash run.sh bash /work/containers/build_overlay.sh
```

Docker native CDI requires a valid generated NVIDIA CDI specification; validate
with `nvidia-ctk cdi list`. `run.sh` maps only the selected index and restricts
resources. `nvidia-smi` validates mapping, while Gazebo camera output validates
rendering. The shared host driver must remain compatible with the image; the
container does not install or replace a host driver.

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
