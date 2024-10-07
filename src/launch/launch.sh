#! /bin/bash
source /opt/ros/humble/setup.bash
source /home/bryn/hyu_ws/install/setup.bash

terminator -x bash -c "ros2 run turtlesim turtlesim_node" &
sleep 1
terminator -x bash -c "source /home/bryn/hyu_ws/install/setup.bash; ros2 run team2_package moving_turtle" &
sleep 1
terminator -x bash -c "source /home/bryn/hyu_ws/install/setup.bash; ros2 run team2_package color_changer" &
sleep 1
terminator -x bash -c "source /home/bryn/hyu_ws/install/setup.bash; ros2 run team2_package hmi" &
sleep 1
terminator -x bash -c "rviz2 -d /home/bryn/hyu_ws/src/rviz/launch.rviz"
