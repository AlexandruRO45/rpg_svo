cmake_minimum_required(VERSION 3.5)
project(vikit_common)

# Set C++ standard and build type
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags (preserve original intent)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include dirs (preserve all original include logic)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Source files
set(SOURCEFILES
  src/atan_camera.cpp
  src/omni_camera.cpp
  src/math_utils.cpp
  src/vision.cpp
  src/performance_monitor.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/pinhole_camera.cpp
  src/homography.cpp
  src/img_align.cpp
)

# Create vikit library
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    sophus
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# =============================================================
# ROS-SPECIFIC LOGIC - only runs if USE_ROS is TRUE
# =============================================================
if(USE_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)

  # Link ROS dependencies
  ament_target_dependencies(${PROJECT_NAME} rclcpp)

  # Add tests
  add_executable(test_vk_common_camera test/test_camera.cpp)
  target_link_libraries(test_vk_common_camera ${PROJECT_NAME})
  ament_target_dependencies(test_vk_common_camera rclcpp)

  add_executable(test_vk_common_triangulation test/test_triangulation.cpp)
  target_link_libraries(test_vk_common_triangulation ${PROJECT_NAME})
  ament_target_dependencies(test_vk_common_triangulation rclcpp)

  add_executable(test_vk_common_patch_score test/test_patch_score.cpp)
  target_link_libraries(test_vk_common_patch_score ${PROJECT_NAME})
  ament_target_dependencies(test_vk_common_patch_score rclcpp)

  # Install rules for ROS
  install(
    DIRECTORY include/
    DESTINATION include
  )

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  install(
    TARGETS
      test_vk_common_camera
      test_vk_common_triangulation
      test_vk_common_patch_score
    DESTINATION lib/${PROJECT_NAME}
  )

  # Export targets for other ROS packages
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(rclcpp Eigen3 OpenCV Sophus)

  ament_package()

endif(USE_ROS)
