cmake_minimum_required(VERSION 3.8)
project(srm1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Message.msg"
  "srv/Service.srv"
  "action/Act.action"
  DEPENDENCIES
   # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()
# Uncomment to start building. Make sure path is ok. 
# include_directories(
#   include $ENV{HOME}/Workspace/ros2_ws/install/behaviortree_cpp/include/
#   include $ENV{HOME}/Workspace/ros2_ws/install/behaviortree_ros2/include/
# )
#add_library(behaviortree_cpp SHARED IMPORTED GLOBAL)
#set_target_properties(behaviortree_cpp PROPERTIES IMPORTED_LOCATION $ENV{HOME}/Workspace/ros2_ws/install/behaviortree_cpp/lib/libbehaviortree_cpp.so)

#add_executable(srm1_node main.cpp)
#target_link_libraries(srm1_node BT::behaviortree_cpp)