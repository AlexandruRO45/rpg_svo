################################################################################
# user build settings

set(TRACE TRUE)
set(HAVE_G2O FALSE)
set(USE_ROS FALSE)
set(DEBUG_OUTPUT TRUE) # Only relevant if build without ROS

################################################################################

set(PROJECT_NAME svo)
cmake_minimum_required(VERSION 3.14)
project(${PROJECT_NAME})
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo, Debug
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Required so that svo_ros vo can find libfast.so
# TODO why?
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Set definitions
if(TRACE)
  add_definitions(-DSVO_TRACE)
endif()
if(HAVE_G2O)
  add_definitions(-DUSE_BUNDLE_ADJUSTMENT)
endif()
if(USE_ROS)
  add_definitions(-DSVO_USE_ROS)
endif()
if(DEBUG_OUTPUT)
  add_definitions(-DSVO_DEBUG_OUTPUT)
endif()

# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
# Set build flags. Set IS_ARM on odroid board as environment variable
set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

find_package(Boost REQUIRED COMPONENTS thread system)
# find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# set(SOPHUS_USE_BASIC_LOGGING OFF CACHE BOOL "Disable Sophus logging")
# set(BUILD_TESTS OFF CACHE BOOL "Disable Sophus tests")

# Add the subdirectories for our local dependencies
# add_subdirectory(extra/vikit_common/Sophus)
add_subdirectory(extra/fast_neon)
add_subdirectory(extra/vikit_common)
# add_subdirectory(extra/vikit_common/eigen)


if(USE_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
else()
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
endif()

include_directories(
  include
  extra/fast_neon/include
  extra/vikit_common/include 
)

# Set link libraries
list(APPEND LINK_LIBS
  ${Boost_LIBRARIES}
  # ${EIGEN3_LIBS}
  # sophus # Link against the target from the subdirectory
  # eigen/Eigen
  fast # Link against the target from the subdirectory
  ${OpenCV_LIBS}
  vikit_common # Link against the target from the subdirectory
)

# Set sourcefiles
list(APPEND SOURCEFILES
  src/config.cpp
  src/depth_filter.cpp
  src/feature_alignment.cpp
  src/feature_detection.cpp
  src/frame.cpp
  src/frame_handler_mono.cpp
  src/frame_handler_base.cpp
  src/initialization.cpp
  src/map.cpp
  src/matcher.cpp
  src/point.cpp
  src/pose_optimizer.cpp
  src/reprojector.cpp
  src/sparse_img_align.cpp)

# Add g2o if available
if(HAVE_G2O)
  find_package(G2O REQUIRED)
  include_directories(
    ${G2O_INCLUDE_DIR}
    /usr/include/suitesparse) # for cholmod
  list(APPEND LINK_LIBS
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} ${G2O_TYPES_SBA} cholmod cxsparse)
  list(APPEND SOURCEFILES src/bundle_adjustment.cpp)
endif(HAVE_G2O)

# Create svo library
add_library(svo SHARED
  ${SOURCEFILES})

if(USE_ROS)
  # Modern CMake target
  add_library(svo::svo ALIAS svo)
endif()

target_link_libraries(svo
  ${LINK_LIBS}
)

if(USE_ROS)
  target_link_libraries(svo
    rclcpp::rclcpp
  )
endif()


################################################################################
# TESTS

add_executable(test_feature_align test/test_feature_alignment.cpp)
target_link_libraries(test_feature_align svo)

add_executable(test_pipeline test/test_pipeline.cpp)
target_link_libraries(test_pipeline svo)

add_executable(test_matcher test/test_matcher.cpp)
target_link_libraries(test_matcher svo)

add_executable(test_feature_detection test/test_feature_detection.cpp)
target_link_libraries(test_feature_detection svo)

add_executable(test_depth_filter test/test_depth_filter.cpp)
target_link_libraries(test_depth_filter svo)

add_executable(test_sparse_img_align test/test_sparse_img_align.cpp)
target_link_libraries(test_sparse_img_align svo)

add_executable(test_pose_optimizer test/test_pose_optimizer.cpp)
target_link_libraries(test_pose_optimizer svo)

if(USE_ROS)
  install(TARGETS ${PROJECT_NAME} EXPORT svo_targets INCLUDES DESTINATION include)
  install(DIRECTORY include/ DESTINATION include)  # Note trailing /
  ament_export_targets(svo_targets)
  ament_export_dependencies(fast rclcpp)
  ament_package()
endif()