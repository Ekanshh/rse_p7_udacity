#!/bin/sh
catkin_dir=/home/workspace/
catkin_src_dir=$catkin_dir/src
project_src_dir=$catkin_src_dir/project_6

xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$project_src_dir/world/my_world.world" &
sleep 5
xterm  -e  "roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5
xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  "roslaunch turtlebot_teleop keyboard_teleop.launch" 


