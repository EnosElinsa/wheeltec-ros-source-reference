#! /bin/bash

### BEGIN INIT

gnome-terminal -- bash -c "ps -aux | grep wheeltec_ros2 | grep -v grep | grep -v qt_ros_test | cut -c 9-16 | xargs kill -INT "

gnome-terminal -- bash -c "ps -aux | grep humble | grep -v grep | grep -v qt_ros_test | cut -c 9-16 | xargs kill -INT "

sleep 5

wait
exit 0


