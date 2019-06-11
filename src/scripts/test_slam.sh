#!/bin/sh

# Launch turtlebot_world.launch file to deploy a turtlebot in my environment
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/HomeServiceRobot_ws/src/map/MyOfficeWorld.world " &
sleep 5

# Launch gmapping_demo.launch or slam_gmapping to perform SLAM
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

# launch view_navigation.launch to observe the map in rviz
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

# launch keyboard_teleop.launch to manually control the robot with keyboard commands
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch "
