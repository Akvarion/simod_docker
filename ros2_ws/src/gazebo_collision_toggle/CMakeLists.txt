cmake_minimum_required(VERSION 3.10)
project(gazebo_collision_toggle)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED) # Gazebo Classic
find_package(rosidl_default_generators REQUIRED)

# Interfacce
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCollisionEnabled.srv"
  DEPENDENCIES builtin_interfaces
)

add_library(toggle_collision_plugin SHARED
  src/toggle_collision_plugin.cpp
)

ament_target_dependencies(toggle_collision_plugin
  rclcpp
)

target_include_directories(toggle_collision_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  ${GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(toggle_collision_plugin
  ${GAZEBO_LIBRARIES}
)

# Collega i typesupport generati
rosidl_target_interfaces(toggle_collision_plugin
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS toggle_collision_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

ament_export_dependencies(rclcpp gazebo rosidl_default_runtime)
ament_package()
