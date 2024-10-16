cmake_minimum_required(VERSION 3.8)
project(teleop_twist_joy_comfy)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/teleop_twist_joy_comfy.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "teleop_twist_joy_comfy::TeleopTwistJoyComfy"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
