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

# Set build flags.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Start with common, cross-platform flags
set(CMAKE_CXX_FLAGS "-Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

# Add platform-specific flags for Linux only
if(UNIX AND NOT APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_LINUX -D_REENTRANT -march=native")
  if(DEFINED ENV{ARM_ARCHITECTURE})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
  else()
    # x86-specific SSE flags for Linux
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse2 -msse3 -mssse3")
  endif()
endif()

# Release flags (note: removed '-fsee' as it's likely a typo and not a standard flag)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# --- PYBIND11 INTEGRATION START ---
add_subdirectory(extra/pybind11)

# Find pybind11 and other dependencies (INSTALLED ON OS)
# find_package(pybind11 CONFIG REQUIRED)
# message(STATUS "Found pybind11 v${pybind11_VERSION}: ${pybind11_INCLUDE_DIRS}")
# --- PYBIND11 INTEGRATION END ---

find_package(Boost REQUIRED COMPONENTS thread system)
# find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Add the subdirectories for our local dependencies
# add_subdirectory(extra/vikit_common/Sophus)
add_subdirectory(extra/fast_neon)
add_subdirectory(extra/vikit_common)
# add_subdirectory(extra/vikit_common/eigen)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
  include
  ${pybind11_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIR}
  extra/fast_neon/include
  extra/vikit_common/include
)

# Set link libraries
list(APPEND LINK_LIBS
  ${Boost_LIBRARIES}
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

# Create svo library (as an object library to be used by other targets)
add_library(svo_static STATIC
  ${SOURCEFILES})

target_compile_options(svo_static PRIVATE -Wno-maybe-uninitialized -Wno-dev -Wno-deprecated-declarations)

# This is required because we are linking this static library into a shared one (the python module).
set_property(TARGET svo_static PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(svo_static
  ${LINK_LIBS}
)

# --- PYBIND11 INTEGRATION START ---
# Create the Python module
# This module includes the binding code and links against the SVO static library
pybind11_add_module(svo_cpp SHARED src/bindings.cpp)
if(PYTHON_INCLUDE_DIR)
  target_include_directories(svo_cpp PRIVATE ${PYTHON_INCLUDE_DIR})
endif()

if(NUMPY_INCLUDE_DIR)
  target_include_directories(svo_cpp PRIVATE ${NUMPY_INCLUDE_DIR})
endif()
target_link_libraries(svo_cpp PRIVATE svo_static ${LINK_LIBS})
# --- PYBIND11 INTEGRATION END ---
################################################################################
# TESTS

# add_executable(test_feature_align test/test_feature_alignment.cpp)
# target_link_libraries(test_feature_align svo_static)

add_executable(test_pipeline test/test_pipeline.cpp)
target_link_libraries(test_pipeline svo_static)

# add_executable(test_matcher test/test_matcher.cpp)
# target_link_libraries(test_matcher svo_static)

# add_executable(test_feature_detection test/test_feature_detection.cpp)
# target_link_libraries(test_feature_detection svo_static)

# add_executable(test_depth_filter test/test_depth_filter.cpp)
# target_link_libraries(test_depth_filter svo_static)

# add_executable(test_sparse_img_align test/test_sparse_img_align.cpp)
# target_link_libraries(test_sparse_img_align svo_static)

# add_executable(test_pose_optimizer test/test_pose_optimizer.cpp)
# target_link_libraries(test_pose_optimizer svo_static)