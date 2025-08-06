# Minimal CMakeLists.txt for Eigen (as a header-only library)
cmake_minimum_required(VERSION 3.10)
project(Eigen3)

# Create an INTERFACE library target. This is used to propagate
# usage requirements (like include directories) to consumers.
add_library(eigen INTERFACE)

# For compatibility with find_package(Eigen3)
add_library(Eigen3::Eigen ALIAS eigen)

# Tell any target that links to 'eigen' to add the current directory
# to its include paths. This allows your code to use #include <Eigen/Dense>.
target_include_directories(eigen INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})