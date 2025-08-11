cmake_minimum_required(VERSION 3.14)
project(svo)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- User Build Options ---
option(SVO_TRACE "Enable tracing for performance monitoring" ON)
option(SVO_HAVE_G2O "Enable g2o for bundle adjustment" OFF)
option(SVO_USE_ROS "Enable ROS integration" OFF)
option(SVO_DEBUG_OUTPUT "Enable debug output (if not using ROS)" ON)

# --- Find/Add Dependencies ---
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(OpenCV REQUIRED)

add_subdirectory(extra/pybind11)
add_subdirectory(extra/fast_neon)
add_subdirectory(extra/vikit_common)

# --- Define SVO Library Source Files ---
set(SVO_SOURCES
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
    src/sparse_img_align.cpp
)

# --- Create the Core Static Library Target ---
add_library(svo_static STATIC ${SVO_SOURCES})
set_property(TARGET svo_static PROPERTY POSITION_INDEPENDENT_CODE ON)

# --- Apply Options, Flags, and Includes to the Static Library ---

# Add preprocessor definitions based on user options
if(SVO_TRACE)
    target_compile_definitions(svo_static PRIVATE SVO_TRACE)
endif()
if(SVO_USE_ROS)
    target_compile_definitions(svo_static PRIVATE SVO_USE_ROS)
endif()
if(SVO_DEBUG_OUTPUT)
    target_compile_definitions(svo_static PRIVATE SVO_DEBUG_OUTPUT)
endif()

# Set compiler flags for the static library
set(BASE_CXX_FLAGS "-Wall;-D_LINUX;-D_REENTRANT;-Wno-unused-variable;-Wno-maybe-uninitialized;-Wno-deprecated-declarations")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|armv8")
    set(ARCH_FLAGS "-O3;-march=native")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(ARCH_FLAGS "-O3;-mfpu=neon;-march=armv7-a")
else() # x86_64
    set(ARCH_FLAGS "-O3;-mmmx;-msse;-msse2;-msse3;-mssse3")
endif()

# Apply all flags to our library target, using correct generator expression syntax
target_compile_options(svo_static PRIVATE
    ${BASE_CXX_FLAGS}
    ${ARCH_FLAGS}
    "$<IF:$<CONFIG:Release>,-fsee;-fomit-frame-pointer;-fno-signed-zeros;-fno-math-errno;-funroll-loops,\>"
)

# Set include directories for the static library
target_include_directories(svo_static PUBLIC
    # Its own headers need to be public for the python module to see them
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link the static library against its dependencies
target_link_libraries(svo_static PUBLIC
    # These are PUBLIC so the dependencies propagate to any target that uses svo_static
    fast
    vikit_common
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
)

# --- Handle g2o dependency (if enabled) ---
if(SVO_HAVE_G2O)
    target_compile_definitions(svo_static PRIVATE USE_BUNDLE_ADJUSTMENT)
    target_sources(svo_static PRIVATE src/bundle_adjustment.cpp)
    find_package(G2O REQUIRED)
    target_include_directories(svo_static PRIVATE ${G2O_INCLUDE_DIR} /usr/include/suitesparse)
    target_link_libraries(svo_static PUBLIC ${G2O_LIBRARIES} cholmod cxsparse)
endif()

# --- Create the Python Module ---
pybind11_add_module(svo_cpp SHARED src/bindings.cpp src/NDArrayConverter.cpp)
if(PYTHON_INCLUDE_DIR)
  target_include_directories(svo_cpp PRIVATE ${PYTHON_INCLUDE_DIR})
endif()

if(NUMPY_INCLUDE_DIR)
  target_include_directories(svo_cpp PRIVATE ${NUMPY_INCLUDE_DIR})
endif()

target_link_libraries(svo_cpp PRIVATE svo_static)

# --- Output Paths ---
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
################################################################################
# TESTS

add_executable(test_feature_align test/test_feature_alignment.cpp)
target_link_libraries(test_feature_align svo_static)

add_executable(test_pipeline test/test_pipeline.cpp)
target_link_libraries(test_pipeline svo_static)

add_executable(test_matcher test/test_matcher.cpp)
target_link_libraries(test_matcher svo_static)

add_executable(test_feature_detection test/test_feature_detection.cpp)
target_link_libraries(test_feature_detection svo_static)

add_executable(test_depth_filter test/test_depth_filter.cpp)
target_link_libraries(test_depth_filter svo_static)

add_executable(test_sparse_img_align test/test_sparse_img_align.cpp)
target_link_libraries(test_sparse_img_align svo_static)

add_executable(test_pose_optimizer test/test_pose_optimizer.cpp)
target_link_libraries(test_pose_optimizer svo_static)