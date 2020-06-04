#! /bin/sh

SOURCE_PATH="/home/workspace/hsbot"
WORLD_FILE="/home/fresh/Documents/learn/robond/hsbot/src/my_robot/worlds/robots_in_apt2.world"

xterm  -e " source ${SOURCE_PATH}; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${WORLD_FILE} " &
sleep 2

xterm  -e " source ${SOURCE_PATH}; roslaunch turtlebot_gazebo gmapping_demo.launch  " &
sleep 1

xterm  -e " source ${SOURCE_PATH}; roslaunch turtlebot_rviz_launchers view_navigation.launch  " &
sleep 1

xterm  -e " source ${SOURCE_PATH}; roslaunch turtlebot_teleop keyboard_teleop.launch  " &
sleep 1