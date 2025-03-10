# Installing Moveit! 2

## Preparing packages
Assumung ROS2 is already installed on the system, make sure you have up to date packages:
```bash
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
```
Moveit 2 should already be present in the workspacefolder and already to be built as per README.

Then create a package, build it (singularly if nothing else changed) and source:
```bash
cd ros2_ws/src
ros2 pkg create my_bt_moveit --build-type ament_cmake --dependencies rclcpp moveit_ros_planning moveit_ros_planning_interface behaviortree_cpp geometry_msgs

cd ..
colcon build --packages-select my_bt_moveit
source install/setup.bash
```

In the following, we assume that `BehaviorTree.CPP` is already installed (it should be the case if the docker is started from the usual git repo). 

---
You can check it via:
```bash
apt-cache search behaviortree
```
If this is not the case, please install it:
```bash
apt update
apt install -y ros-${ROS_DISTRO}-behaviortree-cpp
```  


## Prepare your ROS2 node for the Behavior Tree

This package will be used to prepare the interaction between your Behavior Tree and Moveit! simulating the action via in the Gazebo world.

---

Import into the package your ROS2 code for the Behavior Tree. It should be similar to the following demo example.

### Example code

#### Defining the Action C++ 
In a path similar to `ros2_ws/src/my_bt_moveit/src/move_base_action.cpp` you must have:

```cpp
#include "../include/my_bt_moveit/move_base_action.hpp"

MoveBaseAction::MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList MoveBaseAction::providedPorts() {
    return { 
        BT::InputPort<double>("target_x"),
        BT::InputPort<double>("target_y"),
        BT::InputPort<double>("target_theta")
    };
}

BT::NodeStatus MoveBaseAction::tick() {
    double x, y, theta;
    if (!getInput("target_x", x) || !getInput("target_y", y) || !getInput("target_theta", theta)) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveBaseAction"), "Missing input ports");
        return BT::NodeStatus::FAILURE;
    }

    // Creiamo un nodo ROS2 per pubblicare comandi di velocità
    auto node = rclcpp::Node::make_shared("move_base_action_node");
    auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/right_summit/cmd_vel", 10);

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = x;
    cmd_msg.angular.z = theta;

    RCLCPP_INFO(node->get_logger(), "Moving to x: %.2f, y: %.2f, theta: %.2f", x, y, theta);
    
    // Pubblica il messaggio per far muovere il robot
    cmd_pub->publish(cmd_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(2000));  // Simula il movimento

    return BT::NodeStatus::SUCCESS;
}

```


You can have also the `hpp` file at the path `ros2_ws/src/my_bt_moveit/include/my_bt_moveit/move_base_action.hpp`, with the following:

```cpp
#ifndef MOVE_BASE_ACTION_HPP
#define MOVE_BASE_ACTION_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MoveBaseAction : public BT::SyncActionNode {
public:
    MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

#endif // MOVE_BASE_ACTION_HPP
```

#### Behavior Tree XML structure

You need also to define the `xml` file for the behavior_tree at the path `ros2_ws/src/my_bt_moveit/behavior_trees/behavior_tree.xml`, for example, to move the robotnik base you can have:
```xml
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <MoveBase target_x="1.0" target_y="0.0" target_theta="0.0" />
            <MoveBase target_x="0.0" target_y="0.0" target_theta="1.57" />
            <MoveBase target_x="0.0" target_y="0.0" target_theta="-1.57" />
            <MoveBase target_x="0.0" target_y="0.0" target_theta="0.0" />
        </Sequence>
    </BehaviorTree>
</root>
```

In this example the behavior tree:
- Moves 1 m ahead
- turn robot 90° left
- turn robot 90° right
- stop

#### Defining the ROS2 node to execute the Behavior Tree

Then, you need the Behavior Tree ROS2 node in a path similar to: `ros2_ws/src/my_bt_moveit/src/bt_moveit_node.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>
#include "../include/my_bt_moveit/move_base_action.hpp"

class BehaviorTreeNode : public rclcpp::Node {
public:
    BehaviorTreeNode() : Node("bt_moveit_node") {
        RCLCPP_INFO(this->get_logger(), "Behavior Tree Node Started");

        // Creazione del Behavior Tree Factory
        BT::BehaviorTreeFactory factory;

        // Registriamo il nodo di azione MoveBase
        factory.registerNodeType<MoveBaseAction>("MoveBase");

        // Carichiamo il Behavior Tree dal file XML
        auto tree = factory.createTreeFromFile(
            this->declare_parameter<std::string>("bt_xml_path", 
            "src/my_bt_moveit/behavior_trees/behavior_tree.xml"));

        // Esegui il Behavior Tree con un loop
        while (rclcpp::ok()) {
            BT::NodeStatus status = tree.tickRoot();
            if (status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Behavior Tree Success!");
                break;
            } else if (status == BT::NodeStatus::FAILURE) {
                RCLCPP_ERROR(this->get_logger(), "Behavior Tree Failed!");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviorTreeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Dependencies and Executable

At this point we are almost ready to test the behavior tree.
Check the correctness of the `package.xml` in `src/my_bt_moveit`. It should look similar to this:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_bt_moveit</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="root@todo.todo">root</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>moveit_ros_planning</depend>
  <depend>moveit_ros_planning_interface</depend>
  <depend>behaviortree_cpp</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
Verify all the dependencies are added.

Update also the `CMakeLists.txt` of the package, adding the executables and possibly required dependencies and packages. It should look like this:

```CMake
cmake_minimum_required(VERSION 3.8)
project(my_bt_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(moveit2 REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(bt_moveit_node 
               src/bt_moveit_node.cpp 
               src/move_base_action.cpp)

ament_target_dependencies(bt_moveit_node 
                          rclcpp 
                          geometry_msgs 
                          behaviortree_cpp
                          moveit_ros_planning_interface)

install(TARGETS
        bt_moveit_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```

Finally, build everything and source [build only my_bt_moveit otherwise the system will run out of RAM (if <32 GB)]:
```bash
colcon build --packages-select my_bt_moveit
source install/setup.bash
```

## Test the BT - simulate in Gazebo
Run the Gazebo world, if not yet running:
```bash
ros2 launch ur main.launch.py
``` 

Now you are ready to run the ROS2 Node to execute the behavior tree:
```bash
ros2 run my_bt_moveit bt_moveit_node --ros-args -p bt_xml_path:=src/my_bt_moveit/behavior_trees/behavior_tree.xml
```
Check the path.