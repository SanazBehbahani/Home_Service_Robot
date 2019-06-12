#!/bin/sh

# Launch turtlebot_world.launch to deploy a turtlebot in my environment
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/HomeServiceRobot_ws/src/map/MyOfficeWorld.world " &
sleep 5

# Launch amcl_demo.launch to localize the turtlebot
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/HomeServiceRobot_ws/src/map/MyOfficeMap.yaml initial_pose_a:=-1.57 initial_pose_x:=0 initial_pose_y:=0" &
sleep 5

# Launch view_navigation.launch to observe map in rviz
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

# Launch pick_objects node
xterm -e "rosrun pick_objects pick_objects"
