sudo apt-get install python-catkin-tools python-rosinstall-generator -y
mkdir -p ~/Code/mavlink_ws/src
cd ~/Code/mavlink_ws
catkin init
wstool init src
rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
catkin build
echo "source ~/Code/mavlink_ws/devel/setup.bash" >> ~/.bashrc
