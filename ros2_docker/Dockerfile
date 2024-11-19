FROM ros:humble

ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}


SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
   build-essential \
   cmake \
   curl \
   git-all \
   software-properties-common \
   && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
   && apt-get install terminator wget -y 

RUN apt-get update \
&& apt-get install -y \
ros-${ROS_DISTRO}-gazebo-msgs \
ros-${ROS_DISTRO}-gazebo-ros \
ros-${ROS_DISTRO}-gazebo-ros-pkgs \
ros-${ROS_DISTRO}-ur-robot-driver \
ros-${ROS_DISTRO}-ros2-control \
ros-${ROS_DISTRO}-ros2-controllers  

ARG DEBIAN_FRONTEND=dialog


