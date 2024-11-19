FROM ros:foxy

ENV WS_DIR="/bridge_ws"
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
ros-foxy-gazebo-msgs \
ros-foxy-gazebo-ros \
ros-foxy-gazebo-ros-pkgs \
ros-foxy-tf2-sensor-msgs \
ros-foxy-tf2-geometry-msgs  
# ros-foxy-ros-controllers 

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update \
    && apt install ros-noetic-ros-base  -y 

RUN apt-get update \
   && apt-get install -y \
   ros-noetic-gazebo-msgs \
   ros-noetic-gazebo-ros \
   ros-noetic-gazebo-ros-pkgs \
   ros-noetic-tf2-sensor-msgs \
   ros-noetic-tf2-geometry-msgs 


ARG DEBIAN_FRONTEND=dialog


