#! /bin/bash

### BEGIN INIT

gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py"

gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 run web_video_server web_video_server"

wait
exit 0


