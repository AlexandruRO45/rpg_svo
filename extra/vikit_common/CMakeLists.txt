SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 3.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(USE_ROS False) # Set False if you want to build this package without Catkin

# Set build flags. Set IS_ARM on odroid board as environment variable
if(UNIX AND NOT APPLE)
  SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
ENDIF()
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
# FIND_PACKAGE(Eigen REQUIRED)
# FIND_PACKAGE(Sophus REQUIRED)

set(SOPHUS_USE_BASIC_LOGGING OFF CACHE BOOL "Disable Sophus logging")
set(BUILD_TESTS OFF CACHE BOOL "Disable Sophus tests")
add_subdirectory(Sophus)
add_subdirectory(eigen)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  # ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${Sophus_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/Sophus/sophus
  ${PROJECT_SOURCE_DIR}/eigen/Eigen
)

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  LIST(APPEND INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS})
  catkin_package(
    DEPENDS Eigen OpenCV Sophus
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Set Sourcefiles
LIST(APPEND SOURCEFILES src/atan_camera.cpp
                        src/omni_camera.cpp
                        src/math_utils.cpp
                        src/vision.cpp
                        src/performance_monitor.cpp
                        src/robust_cost.cpp
                        src/user_input_thread.cpp
                        src/pinhole_camera.cpp
                        src/homography.cpp
                        src/img_align.cpp)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  # ${Sophus_LIBRARIES}
  sophus
  ${catkin_LIBRARIES})

IF(USE_ROS)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES})
ENDIF()


################################################################################
# Create the vikit_commonConfig.cmake file for other cmake projects.
IF(NOT USE_ROS)
  SET(VIKIT_COMMON_LIB_PATH $<TARGET_FILE:vikit_common>)
  SET(vikit_common_LIBRARIES ${FULL_LIBRARY_NAME} )
  SET(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
  SET(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE )
  export( PACKAGE vikit_common )

  INSTALL(DIRECTORY include/vikit DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
ENDIF()