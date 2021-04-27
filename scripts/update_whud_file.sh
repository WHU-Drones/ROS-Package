mkdir -p ~/Code/whud_ws/src
cd ~/Code/whud_ws/src
catkin_init_workspace
cd ~/Code/ROS-Package
rm -rf ~/Code/whud_ws/src/whud*
cp -rf whud* ~/Code/whud_ws/src
cd ~/Code/whud_ws
catkin_make
