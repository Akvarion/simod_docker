# SIMOD Set-up

## Running both ROS1 and ROS2 with Bridge
### 1. Prepare docker-compose
Assuming all the necessary dependencies are met, the file docker compose must be modified in order to result in a working environment.

**Lines 23, 44 and 70 must be edited with the correct path ( <user_path_to_directory> ); paths must be absolute.**

Once the path is set up correctly, run the following command to avoid temporarily rewriting the the one in the repo once you commit the changes:

```bash
git update-index --assume-unchanged docker-compose-gui.yml
```


### 1. Building
 
Build the Docker container with
 
```bash
docker compose -f docker-compose-gui.yml build
```
Should a permission denied error appear, that is because the current user has no sudo permissions granted for docker. Simply adding `sudo` before the command should fix this issue.
### 2. Starting
 
Allow the containers to display contents on your host machine by typing
 
```bash
xhost +local:root
```
Then start the setup with:
```bash
docker compose -f docker-compose-gui.yml up
```
 
Terminator GUI will open up with the Docker container running inside it and the ros1 and ros2 environments already sourced.

#### 2.1 Starting a single container
To start a single container, simply run the following (change the `<options>` into one of the followings: `ros1_simod`, `bridge_simod`, `ros2_simod`):
```bash
docker compose -f docker-compose-gui.yml up <options>
```
### 2.2 Nvidia GPU hardware acceleration
Some machines may use a setup with a dual GPU combo (Integrated + Dedicated Nvidia Cards). To use the Nvidia graphics card as the rendering device, a few requirements must be met:
#### 2.2.1 Nvidia Container Toolkit
To install this tool, you need to:

##### Configure the production repository
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
##### Update
```bash
sudo apt-get update
```
##### Install Nvidia Container Toolkit
```bash
sudo apt-get install -y nvidia-container-toolkit
```
##### Configure Docker
```bash
sudo nvidia-ctk runtime configure --runtime=docker
```
##### Restart Docker Service
```bash
sudo systemctl restart docker
```

##### Uncomment lines 9,14,15,56,60,61 in docker-compose-gui.ymls

Further information can be found at the official [Nvidia Container Toolkit installation page](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

### 3. ROS2 Simulation

At first start ( with `docker compose -f docker-compose-gui.yml up ros2_simod`), it will be necessary to build the workspace:

```bash
cd ./src
rosdep install -r --from-paths . --ignore-src -y
cd ..
MAKEFLAGS="-j4 -l1" colcon build --symlink-install --executor sequential
pip install trimesh pycollada
apt install ros-humble-tf-transformations
```
These commands allow the resolution of dependencies and compilation of the workspace, including Moveit2 Framework. 
Note: during compilation of "moveit_simple_controller_manager", an error regarding the lack of a package may appear:

"simod_moveit_action_controller: package not found". The package is present, but for some reason not yet built.
Running:
```bash
colcon build --symlink-install --packages-select simod_moveit_action_controller
```
should fix this issue.
After building, to start the simulation environment, simply:
```bash
source install/setup.bash
ros2 launch ur main.launch.py usecase:=1_1
```
The argument `usecase` defines which of the usecases considered is loaded into gazebo. There are 6 possible combinations:
```bash
usecase:=1_1
usecase:=1_2
usecase:=2_1
usecase:=2_2
usecase:=3_1
usecase:=3_2
```
This is done to the reflect the possibility of 2 layers of packages to be picked, as the structure ceiling changes. 
Since the amount of messages flooding the console will be considerable, it is advisable to redirect all output to a log file with `&> userlogname.log`.
Also, it is possible to launch a single robot in the simulation with:

```bash
ros2 launch ur new_single.launch.py robot:=left gazebo:=true
```
the arguments `robot` and `gazebo` select which robot is to be loaded and if the simulation tools are needed to be launched. The argument `gazebo` must be true, otherwise gazebo and rviz won't load; also, at the moment, only the `left` robot will not cause Moveit to complain in the logs.

#### Bridge Container
This container operates the ros1 bridge. It has to be compiled.
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


#### ROS1_DOCKER CONTAINER

*solo la prima volta che si compila il workspace

```bash
colcon build
```
```bash
. install/setup.bash
roslaunch summit_xl_gazebo parameter.launch 
roslaunch summit_xl_gazebo environment.launch
```



### 4. Useful topics

- `${NAMESPACE}/odom` : da cui si ricava la posizione della base rispetto al mondo.
- `${NAMESPACE}/cmd_vel` : controllo in velocità 

Per pubblicare da terminale, c'è un problema di autocompletamento del messaggio:

ex. `ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist msg`

(`ros2 topic pub ${NOME_TOPIC} ${TIPO DEL MESSAGGIO} ${MSG}`)

per ottenere il messaggio, Tab+si vede la prima lettera del messaggio+"prima lettere+Tab

## Running only ROS2 environment

If not yet built, run:

```bash
sudo docker compose -f docker-compose-gui.yml build
```

Then, allow the containers to display contents on your host machine by typing
 
```bash
xhost +local:root
```
### Starting a single container
To start only the container for `ros2`, run the following:
```bash
sudo docker compose -f docker-compose-gui.yml up ros2_simod
```

To restart the previous session use instead:

```bash
sudo docker start -ai simod_docker-ros2_simod-1
```
using the appropriate name of the container you created. You can check it with:
```bash
sudo docker ps -a
```

Now a new terminal running the docker container prepared with ROS2 and Gazebo should appear. The current position correspond to the ROS2 workspace `ros2_ws`.
Now, we can run the software inside this environment.

**Only The first time** you compile the workspace with:
```bash
MAKEFLAGS="-j4 -l1" colcon build --symlink-install --executor sequential
```
This will also compile and Install Moveit2.
Before starting the scripts, you need to source the bash files:

```bash
# . install/setup.bash
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# . /usr/share/gazebo/setup.bash

source ros2_setup.bash
source install/setup.bash #allows autocomplete
```

Finally, we can run the application in the Gazebo environment:

```bash
ros2 launch ur main.launch.py
```

You should see the virtual space with two robots (mobile base and arms) facing each other and stopped in position (0,0,0). 
If the robots do not spawn correctly, a solution can be to delete the build folder and rebuild the whole workspace from scratch.

### Testing the application
In order to try the application you can send a ros message to the topic corresponding to a robot and see if it responds correctly in the gazebo environment.
To do so, open a new tab of the terminal running the container with ros2 (right click > new tab) and use the new one to read/write on the topics corresponding to the robot movements.  
The command can be something similar to:

```bash
# moving the base
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -1
```
ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}"

ros2 topic pub /right_summit/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"





ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

or

```bash
# moving the arm
ros2 topic pub /left/ur_left_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.02, 0.01, 0.03, 0,0,0.1]}"

ros2 topic pub /right/ur_right_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.02, 0.01, 0.03, 0,0,0.1]}"
```
Data: [Rotazione della base, Inclinazione In Avanti (II giunto), Inclinazione in Avanti II (III Giunto), Rotazione X Pinza (IV Giunto), Rotazione Z Pinza (V Giunto), Rotazione Y Pinza (VI Giunto)]

Remember to use the `""`.

Some commands to try out:
```bash
ros2 topic pub /right/ur_right_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [-0.01, -0.01, 0.0, 0,0,0]}"


ros2 topic pub /right/ur_right_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.0, -0.05, 0.1, 0,0,0]}"

ros2 topic pub /right/ur_right_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0,0,0,0,0,0]}"



ros2 topic pub /left/ur_left_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.01, 0.01, 0.05, 0,0.1,0]}"

ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"



ros2 topic pub /left/ur_left_joint_group_vel_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0,0,0,0,0,0]}"

ros2 topic pub /left_summit/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

```
git clone -b humble https://github.com/moveit/moveit_task_constructor.git
git clone https://github.com/ros_planning/geometric_shapes.git
git clone https://github.com/moveit/moveit_visual_tools.git
git clone -b humble https://github.com/moveit/moveit2_tutorials.git
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/ros_controls/realtime_tools.git
git clone -b humble https://github.com/ros2/ros2_control.git
git clone https://github.com/ros2/ros2_control_cmake.git
git clone https://github.com/ros2/ros2_controllers.git
git clone https://github.com/ros2/rosparam_shortcuts.git
```