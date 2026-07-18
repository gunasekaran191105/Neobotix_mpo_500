cmake_minimum_required(VERSION 3.8)
project(neobotix_mpo_500)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find basic ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Find Navigation 2 & Plugin packages
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

# --- BUILD THE PSO PLUGIN LIBRARY ---
add_library(neobotix_pso_planner SHARED src/pso_planner.cpp)

# Tell the compiler where to find the .hpp file
target_include_directories(neobotix_pso_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(neobotix_pso_planner 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  nav2_core 
  nav2_costmap_2d 
  nav2_util 
  pluginlib
)
# Export the plugin XML
pluginlib_export_plugin_description_file(nav2_core pso_plugin.xml)

# Install shared directories (including worlds and maps)
install(
  DIRECTORY config launch maps meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/mecanum_drive_node.py
  scripts/scan_frame_fix_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the PSO Plugin Library and XML
install(TARGETS neobotix_pso_planner
  DESTINATION lib
)
install(FILES pso_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
