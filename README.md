## 1. Building
 
Build the Docker container with
 
```shell
docker compose -f docker-compose-gui.yml build
```
 
## 2. Starting Container
 
Allow the container to display contents on your host machine by typing
 
```bash
xhost +local:root
```
 
IMPORTANT: Modify the path of the current directory in the docker-compose-gui.yml
```shell
- type: bind
source: /home/miriam/Scrivania/ros_bridge_docker/
```
 
```shell
 
Inside the docker folder, start the container
```shell
docker compose -f docker-compose-gui.yml up
```
 
Terminator GUI will open up with the Docker container running inside it and the ros1 and ros2 environments already sourced.
 
 
## 3. Running

BRIDGE_DOCKER

2° Terminale (solo quando si builda la prima volta il workspace)
```bash
colcon build --symlink-install --packages-skip ros1_bridge
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

1° Terminale

```bash
source /opt/ros/noetic/setup.bash
roscore 
```

3° Terminale
```bash
# source /opt/ros/noetic/setup.bash
# . install/setup.bash
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source bridge_setup.bash


* prima di questo comando eseguire ROS1_DOCKER
ros2 run ros1_bridge parameter_bridge
```


ROS1_DOCKER

*solo la prima volat che si builda il workspace

```bash
colcon build
```
```bash
. install/setup.bash
roslaunch summit_xl_gazebo parameter.launch 
roslaunch summit_xl_gazebo environment.launch
```

ROS2_DOCKER

1° Terminale

*solo la prima volat che si builda il workspace
```bash
colcon build --symlink-install 
```

2° Terminale

```bash
# . install/setup.bash
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# . /usr/share/gazebo/setup.bash

source ros2_setup.bash
ros2 launch ur main.launch.py
```

