#!/bin/bash

#. devel/setup.bash
roslaunch bstars_sim bstars_sim.launch gmapping:=true &
sleep 10
xterm -hold -e roslaunch turtlebot_teleop keyboard_teleop.launch