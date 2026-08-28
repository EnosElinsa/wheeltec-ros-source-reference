#! /bin/bash

### BEGIN INIT

gnome-terminal -- bash -c "source /opt/ros/galactic/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch wheeltec_mic_ros2 mic_init.launch.py"

gnome-terminal -- bash -c "source /opt/ros/galactic/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch wheeltec_mic_ros2 base.launch.py"

wait
exit 0


