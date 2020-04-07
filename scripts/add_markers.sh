#!/bin/sh
catkin_dir=/home/workspace/
catkin_src_dir=$catkin_dir/src
project_src_dir=$catkin_src_dir/rse_p7_udacity

xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$project_src_dir/world/my_world.world" &
sleep 5
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$project_src_dir/my_map/my_map.yaml" &
sleep 5
xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm  -e  "rosrun add_markers test_addmarkers"