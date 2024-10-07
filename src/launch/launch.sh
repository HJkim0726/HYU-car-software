#! /bin/bash

SETUP_BASH_PATH="../../install/setup.bash"

source /opt/ros/humble/setup.bash
source $SETUP_BASH_PATH



terminator -x bash -c "ros2 run turtlesim turtlesim_node" &
sleep 1
terminator -x bash -c "source $SETUP_BASH_PATH; ros2 run team2_package moving_turtle" &
sleep 1
terminator -x bash -c "source $SETUP_BASH_PATH; ros2 run team2_package color_changer" &
sleep 1
terminator -x bash -c "source $SETUP_BASH_PATH; ros2 run team2_package hmi" &
sleep 1
terminator -x bash -c "rviz2"
