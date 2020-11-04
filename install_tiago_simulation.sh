# From http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/TiagoSimulation

mkdir ~/tiago_public_ws
cd ~/tiago_public_ws

curl https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/kinetic-devel/tiago_public.rosinstall -o tiago_public.rosinstall

rosinstall src /opt/ros/kinetic tiago_public.rosinstall

sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src --rosdistro kinetic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup slam_toolbox"

sudo apt-get update
sudo apt-get install python-catkin-tools

source /opt/ros/kinetic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0

source ./devel/setup.bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=titanium
