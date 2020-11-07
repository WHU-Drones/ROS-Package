#!/bin/bash

sudo sh -c 'echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

code_name=$(lsb_release -sc)

if [ "$code_name" = "xenial" ]; then
    ros_version="kinetic"
elif [ "$code_name" = "bionic" ]; then
    ros_version="melodic"
else
    echo "this bash does not support "$code_name
    exit
fi

sudo apt install ros-${ros_version}-desktop-full
sudo apt install python-rosdep
sudo rosdep init
rosdep update
echo "source /opt/ros/${ros_version}/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential ros-${ros_version}-usb-cam ros-${ros_version}-serial 
