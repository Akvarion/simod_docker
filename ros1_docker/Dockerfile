FROM osrf/ros:noetic-desktop

ENV WS_DIR="/ros1_ws"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
   build-essential \
   cmake \
   git-all \
   software-properties-common \
   && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
   && apt-get install terminator wget -y 

RUN apt-get update \
   && apt-get install -y \
      python3-pip \
      python3-rosdep \
      python3-rosinstall \
      python3-rosinstall-generator \
      python3-wstool \
      python3-vcstool \
      python3-catkin-tools \
      python3-colcon-common-extensions \
      && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
   && apt-get install -y \
      ros-${ROS_DISTRO}-diagnostic-updater \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \ 
      ros-${ROS_DISTRO}-tf2-sensor-msgs \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-rviz \
      ros-${ROS_DISTRO}-ros-controllers \ 
      ros-${ROS_DISTRO}-robot-localization \  
      ros-${ROS_DISTRO}-mavros* \
      ros-${ROS_DISTRO}-gmapping \
      ros-${ROS_DISTRO}-map-server \
      ros-${ROS_DISTRO}-amcl \
      ros-${ROS_DISTRO}-gazebo-ros \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      ros-${ROS_DISTRO}-gazebo-ros-control \
      ros-${ROS_DISTRO}-moveit* \
      ros-${ROS_DISTRO}-industrial-robot-status-interface \
      ros-${ROS_DISTRO}-scaled-controllers \
      ros-${ROS_DISTRO}-ur-robot-driver \
      ros-${ROS_DISTRO}-ur-calibration \
      ros-${ROS_DISTRO}-imu-complementary-filter \
      && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y \
    lsb-release 

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN apt-get update \
    && apt-get install -y \
    libignition-plugin-dev 

ARG DEBIAN_FRONTEND=dialog


