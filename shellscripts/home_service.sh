#! /bin/sh

SOURCE_PATH="/home/workspace/hsbot"
WORLD_FILE="${SOURCE_PATH}/src/my_robot/worlds/robots_in_apt2.world"
MAP_FILE="${SOURCE_PATH}/src/map/my_map.yaml"
RVIZ_CONF="${SOURCE_PATH}/src/rvizConfig/homeservice.rviz"

xterm  -e " source ${SOURCE_PATH}/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${WORLD_FILE} " &
sleep 5

xterm  -e " source ${SOURCE_PATH}/devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${MAP_FILE} " &
sleep 1

xterm  -e " rosrun rviz rviz -d  ${RVIZ_CONF} " &
sleep 5

xterm  -e " source ${SOURCE_PATH}/devel/setup.bash; rosrun add_markers add_markers " &
sleep 1

xterm  -e " source ${SOURCE_PATH}/devel/setup.bash; rosrun pick_objects pick_objects " &
sleep 1

