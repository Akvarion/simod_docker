cmake_minimum_required(VERSION 3.8)
project(ur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
# find_package(depthai_descriptions REQUIRED)
find_package(ament_cmake_python REQUIRED)
#ament_python_install_module(${PROJECT_NAME})
install(
  DIRECTORY launch xacro rviz config
  DESTINATION share/${PROJECT_NAME}
)
# Python stuff
install(PROGRAMS ur/joint_state_merger.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ur/gazebo_scene_sync.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ur/gazebo_moveit_bridge.py DESTINATION lib/${PROJECT_NAME})
# install(
#   PROGRAMS dd.py
#   DESTINATION lib/${PROJECT_NAME}
# )


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
