# navigation_modify
You can use these files for robot simulation or use the recorded rosbag for global cost map construction and path planning.

Before using it, we suggest that you create a new ROS workspace, then put these two folders in the /src directory of the new workspace, then run catkin_make to compile.

- Robot simulation
(1) Make sure you have set up the gazebo simulation environment
(2) Run the following code in order:
```shell
roslaunch mrobot_gazebo mrobot_laser_nav_gazebo.launch
roslaunch mrobot_navigation exploring_slam_demo.launch
```
(3) Then you can control the car in the simulation environment and see the built map in rviz.

- Use the recorded rosbag
(1) Run the following code:
```shell
roslaunch mrobot_navigation get_costmap.launch bag_filename=${YOUR ROSBAG PAHT}
```
Please replace the sentence ${YOUR ROSBAG PAHT} with your rosbag path, Then you can see the global cost map constructed in rviz.
