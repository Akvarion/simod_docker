cmake_minimum_required(VERSION 3.10)
project(gazebo_link_gravity_toggle)

# ====== Dipendenze ======
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)  # Gazebo Classic
find_package(rosidl_default_generators REQUIRED)

# ====== Interfacce (srv) ======
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLinkGravity.srv"
  DEPENDENCIES builtin_interfaces
)

# ====== Plugin (world plugin) ======
add_library(toggle_link_gravity_plugin SHARED
  src/toggle_link_gravity_plugin.cpp
)

# Dipendenze ament sul plugin
ament_target_dependencies(toggle_link_gravity_plugin
  rclcpp
)

# Include e link di Gazebo
target_include_directories(toggle_link_gravity_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(toggle_link_gravity_plugin
  ${GAZEBO_LIBRARIES}
)

# Collega i typesupport generati alla tua libreria (evita errori di target mancanti)
rosidl_target_interfaces(toggle_link_gravity_plugin
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ====== Install ======
install(TARGETS toggle_link_gravity_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# ====== Export ======
ament_export_dependencies(rclcpp gazebo rosidl_default_runtime)
ament_package()
