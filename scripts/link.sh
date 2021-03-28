#!/bin/bash
# sleep 10
sudo chmod 777 /dev/ttyTHS0
sudo chmod 777 /dev/ttyTHS2

source /opt/ros/melodic/setup.bash
source /home/nvidia/Code/catkin_ws/devel/setup.bash
source /home/nvidia/Code/cartographer_ws/install_isolated/setup.bash

roslaunch whud_union whud_union.launch
