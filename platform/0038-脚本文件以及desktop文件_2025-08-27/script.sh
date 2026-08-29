#! /bin/bash
xhost +
sleep 5

#open stm32_serial
xfce4-terminal -e " bash -c 'source /opt/ros/humble/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py; exec bash'"
sleep 2

#open keyboard
xfce4-terminal -e " bash -c 'source /opt/ros/humble/setup.bash;source /home/wheeltec/wheeltec_ros2/install/setup.bash;ros2 run wheeltec_robot_keyboard wheeltec_keyboard; exec bash'"
sleep 2






wait

