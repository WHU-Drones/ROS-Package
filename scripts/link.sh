#!/bin/bash
sleep 5
sudo chmod 777 /dev/ttyTHS0
python cargo_gpio_init.py 

source /opt/ros/melodic/setup.bash
source /home/nvidia/Code/catkin_ws/devel/setup.bash
source /home/nvidia/Code/cartographer_ws/install_isolated/setup.bash
source /home/nvidia/Code/mavros_ws/devel/setup.bash


# roslaunch whud_union whud_union.launch
