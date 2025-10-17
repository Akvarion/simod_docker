cmake_minimum_required(VERSION 3.8)
project(simod_moveit_action_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the action messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MultiDofFollowJointTrajectory.action"
  DEPENDENCIES std_msgs trajectory_msgs
)
# Make sure interface targets are available
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(simod_mac src/simod_mac.cpp)
ament_target_dependencies(simod_mac
  rclcpp
  rclcpp_action
  geometry_msgs
  trajectory_msgs
  moveit_core
)
target_link_libraries(simod_mac "${typesupport_target}")

target_include_directories(simod_mac PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simod_mac PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS simod_mac
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)


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
