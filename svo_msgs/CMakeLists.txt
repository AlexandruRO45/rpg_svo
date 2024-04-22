cmake_minimum_required(VERSION 3.14)
project(svo_msgs)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.27")
  # Avoid error messages in CLion + ROS2 Humble, can be removed for ROS2 Iron
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate ROS messages
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DenseInput.msg
  msg/Feature.msg
  msg/Info.msg
  msg/NbvTrajectory.msg
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

ament_auto_package()
