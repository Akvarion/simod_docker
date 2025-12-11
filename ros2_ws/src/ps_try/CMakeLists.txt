cmake_minimum_required(VERSION 3.8)
project(ps_try)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(ps_try src/ps_try.cpp)

ament_target_dependencies(ps_try moveit_task_constructor_core rclcpp)
target_link_libraries(ps_try yaml-cpp)

target_include_directories(ps_try PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ps_try PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ps_try DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install XML behavior tree files
install(DIRECTORY scripts/behaviortree_XML_demo
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.xml"
)

# Install Python scripts so they are available as executables after colcon build
install(PROGRAMS
  scripts/move_demo.py
  scripts/pick_and_place_demo.py
  scripts/return_robots_home.py
  scripts/bt_ros2_demo.py
  scripts/test_sim_time.py
  scripts/bt_ros2_demo-xml.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
