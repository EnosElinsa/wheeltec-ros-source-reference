ARG BASE_IMAGE=public.ecr.aws/ubuntu/ubuntu:22.04@sha256:82d7c4c4039a00dcd010fb175e136bcec3c79ce25c334666b09a483d2953876b
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 ROS_DISTRO=humble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg locales git build-essential cmake pkg-config \
    python3-pip python3-pytest python3-numpy python3-yaml python3-psutil \
    python3-opencv python3-matplotlib unzip libceres-dev libeigen3-dev libboost-all-dev \
    libopencv-dev libpcl-dev liboctomap-dev libsqlite3-dev libyaml-cpp-dev \
    libegl1 libgl1 libglvnd0 mesa-utils && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL --retry 3 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main' \
      > /etc/apt/sources.list.d/ros2.list && \
    curl -fsSL --retry 3 https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-rosdep \
      ros-humble-ros-base ros-humble-ros-gz ignition-fortress \
      ros-humble-navigation2 ros-humble-nav2-bringup \
      ros-humble-nav2-mppi-controller ros-humble-nav2-smac-planner \
      ros-humble-robot-localization ros-humble-rtabmap-ros \
      ros-humble-cv-bridge ros-humble-image-transport-plugins \
      ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rosbag2-storage-mcap && \
    mkdir -p /opt/wheeltec/evidence && \
    dpkg-query -W -f='${Package}\t${Version}\n' > /opt/wheeltec/evidence/packages.tsv && \
    rm -rf /var/lib/apt/lists/*

ENV QT_QPA_PLATFORM=offscreen MPLBACKEND=Agg NVIDIA_VISIBLE_DEVICES=void
WORKDIR /work
COPY entrypoint.sh /opt/wheeltec/entrypoint.sh
RUN chmod +x /opt/wheeltec/entrypoint.sh
ENTRYPOINT ["/opt/wheeltec/entrypoint.sh"]
CMD ["bash"]
