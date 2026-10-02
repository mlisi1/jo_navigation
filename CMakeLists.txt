cmake_minimum_required(VERSION 3.5.1)
project(jo_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(jo_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# dynamic_obstacle_layer: costmap plugin marking the obstacles tracked by
# onboard_detector_v2 (and GLIM's bbox rejection), ported from jo-sim_lv-dot
add_library(dynamic_obstacle_layer SHARED
  src/dynamic_obstacle_layer.cpp)
target_compile_features(dynamic_obstacle_layer PUBLIC cxx_std_17)
ament_target_dependencies(dynamic_obstacle_layer
  rclcpp nav2_costmap_2d pluginlib jo_msgs geometry_msgs visualization_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(
  DIRECTORY config launch rviz missions
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(scripts)

install(
  PROGRAMS  scripts/send_gps_waypoints.py
            scripts/csv_to_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  dynamic_obstacle_layer
  DESTINATION lib
)

ament_package()
