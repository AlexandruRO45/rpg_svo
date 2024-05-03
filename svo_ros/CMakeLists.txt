set(PROJECT_NAME svo_ros)
cmake_minimum_required (VERSION 3.14)
project(${PROJECT_NAME})
set(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo, Debug

# Required so that svo_ros vo can find libfast.so
# TODO why?
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Set build flags. Set IS_ARM on odroid board as environment variable
set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(fast REQUIRED)  # TODO svo should export this dep
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

# Create visualizer library
ament_auto_add_library(svo_visualizer STATIC
  src/visualizer.cpp)
target_link_libraries(svo_visualizer
  ${Boost_LIBRARIES}
  Eigen3::Eigen
  ${fast_LIBRARIES}  # TODO svo should export this dep
  ${OpenCV_LIBS}
  Sophus::Sophus)
                                 
# Create Executables                      
ament_auto_add_executable(vo_node
  src/vo_node.cpp)
target_link_libraries(vo_node
  ${fast_LIBRARIES}  # TODO svo should export this dep
  svo_visualizer)

# Not ported yet
# ament_auto_add_executable(benchmark src/benchmark_node.cpp)
# target_link_libraries(benchmark svo_visualizer)

ament_auto_package(INSTALL_TO_SHARE config launch param)
