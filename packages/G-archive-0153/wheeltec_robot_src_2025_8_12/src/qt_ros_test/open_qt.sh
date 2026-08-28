#! /bin/bash

### BEGIN INIT

gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/wheeltec/wheeltec_robot/devel/setup.bash;roscore"

gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/wheeltec/wheeltec_robot/devel/setup.bash;source /home/wheeltec/wheeltec_lidar/devel/setup.bash;rosrun qt_ros_test qt_ros_test"

wait
exit 0


