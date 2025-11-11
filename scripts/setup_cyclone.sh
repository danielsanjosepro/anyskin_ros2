export ROS_DOMAIN_ID=100 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_NETWORK_INTERFACE=enx607d0937fb24
export CYCLONEDDS_URI=$HOME/ros2_ws/src/anyskin_ros2/config/cyclone_config.xml

ros2 daemon stop
ros2 daemon start
