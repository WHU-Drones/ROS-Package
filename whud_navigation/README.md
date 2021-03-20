# whud_navigation
You can use the recorded rosbag for global cost map construction and path planning.

Before using it, we suggest that you create a new ROS workspace, then put the folder in the /src directory of the new workspace, then run catkin_make to compile.

- How to Use the recorded rosbag

Run the following code:

```shell
roslaunch whud_navigation get_costmap.launch bag_filename:=${YOUR ROSBAG PAHT}
```

Please replace the sentence ${YOUR ROSBAG PAHT} with your rosbag path, then you can see the global cost map constructed in rviz.
