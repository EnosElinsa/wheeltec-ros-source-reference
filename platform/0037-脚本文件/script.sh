echo "begin camera work"

DISPLAY=${DISPLAY:-unix:0}

docker start ros1

docker update --restart=always ros1

container_id=$(docker ps -qf "name=ros1")

docker exec -it -e DISPLAY=$DISPLAY $container_id bash -c "source '/opt/ros/melodic/setup.bash' && \
source '/home/wheeltec/wheeltec_robot/devel/setup.bash' && \
roslaunch simple_follower line_follower.launch && \
bash"

sleep 2
echo "success run"
