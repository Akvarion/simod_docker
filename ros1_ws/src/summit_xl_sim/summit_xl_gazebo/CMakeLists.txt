cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_gazebo)


find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_srvs
    std_msgs
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    gazebo_ros
    joint_state_controller
    velocity_controllers
    gazebo_ros_control
    geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_srvs
    std_msgs
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
)


#############
## Install ##
#############


install(
  DIRECTORY launch worlds rviz src xacro config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


