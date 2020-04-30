#!/bin/bash

#generates rosserial libraries using rosserial_stm32 package
rm -rf Inc/actionlib
rm -rf Inc/actionlib_msgs
rm -rf Inc/actionlib_tutorials
rm -rf Inc/bond
rm -rf Inc/control_msgs
rm -rf Inc/diagnostic_msgs
rm -rf Inc/duration.cpp
rm -rf Inc/dynamic_reconfigure
rm -rf Inc/geometry_msgs
rm -rf Inc/map_msgs
rm -rf Inc/nav_msgs
rm -rf Inc/nodelet
rm -rf Inc/ros
rm -rf Inc/roscpp
rm -rf Inc/roscpp_tutorials
rm -rf Inc/rosgraph_msgs
rm -rf Inc/ros.h
rm -rf Inc/rospy_tutorials
rm -rf Inc/rosserial_msgs
rm -rf Inc/sensor_msgs
rm -rf Inc/shape_msgs
rm -rf Inc/smach_msgs
rm -rf Inc/std_msgs
rm -rf Inc/std_srvs
rm -rf Inc/stereo_msgs
rm -rf Inc/STM32Hardware.h
rm -rf Inc/tetra_ros
rm -rf Inc/tf
rm -rf Inc/tf2_msgs
rm -rf Inc/time.cpp
rm -rf Inc/topic_tools
rm -rf Inc/trajectory_msgs
rm -rf Inc/turtle_actionlib
rm -rf Inc/turtlesim
rm -rf Inc/urg_node
rm -rf Inc/visualization_msgs
rosrun rosserial_stm32 make_libraries.py . tetra_ros/CompactOdom