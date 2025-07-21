# Minimal CMakeLists.txt for Sophus (as a header-only library)
cmake_minimum_required(VERSION 3.10)
project(Sophus)

# Create an INTERFACE library target for Sophus.
add_library(sophus INTERFACE)

# Add an alias for modern CMake practice (Sophus::Sophus).
add_library(Sophus::Sophus ALIAS sophus)

# Tell any target that links to 'sophus' to add the current directory
# to its include paths. This enables #include <sophus/se3.hpp>.
target_include_directories(sophus INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# Sophus depends on Eigen. Since your project also includes Eigen,
# we declare that dependency here. This automatically adds Eigen's
# include path to any project that uses Sophus.
target_link_libraries(sophus INTERFACE eigen)