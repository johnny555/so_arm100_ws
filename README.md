# so_arm100_ws

Workspace for experimenting with using the SO-ARM100 and ROS2 and Moveit. 

To get a ROS2 controller, we are using the topic based ros2_control library from https://github.com/PickNikRobotics/topic_based_ros2_control

## so_arm100 package 

This is a package with urdf and launch files for running ros2 nodes with the so_arm100.

## st3215_control serial

This is a package for using serial to control the st3215. 

## st3215_control microros

This package can also use the st3215 control through an esp32 servo driver board running microros. 

See this repo for the source code for that https://github.com/johnny555/st3215_microros/tree/main
