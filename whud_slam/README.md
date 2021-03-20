# whud_slam
These files are used to configure the cartographer, after configuration, it can use scan and imu data to build maps.

Before using, you need to complete the following steps:

- (1) Copy the whud_carto_2d.launch file to the $(find cartographer_ros)/launch directory.
- (2) Copy the whud_rplidar.lua file to the $(find cartographer_ros)/configuration_files directory.
- (3) Copy the whud_rplidar.urdf file to the $(find cartographer_ros)/urdf directory.
- (4) Run catkin_make_isolated --install --use-ninja to recompile your cartographer.
- (5) Make sure you have cloned the whud_laser_filter package in the project and have put it in a workspace.

When you run the whud_carto_2d.launch file, please note that you need to set the bag_filename parameter to your rosbag path.

