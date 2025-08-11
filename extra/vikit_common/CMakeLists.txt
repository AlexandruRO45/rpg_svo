cmake_minimum_required(VERSION 3.8)
project(vikit_common)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Handle Dependencies ---
# These commands find system packages or add libraries from subdirectories.
set(SOPHUS_USE_BASIC_LOGGING OFF CACHE BOOL "Disable Sophus logging")
set(BUILD_TESTS OFF CACHE BOOL "Disable Sophus tests")
find_package(OpenCV REQUIRED)
add_subdirectory(Sophus)
add_subdirectory(eigen)

# --- Define Source Files ---
set(VIKIT_SOURCES
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

# --- Create Library Target ---
add_library(${PROJECT_NAME} SHARED ${VIKIT_SOURCES})

# --- Set Compiler Flags per Target ---
# Base flags for all builds
set(BASE_CXX_FLAGS
    -Wall
    -D_LINUX
    -D_REENTRANT
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-unknown-pragmas
)

# Architecture-specific flags
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|armv8")
    message(STATUS "Configuring for aarch64/armv8.")
    set(ARCH_FLAGS "-O3" "-march=native")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    message(STATUS "Configuring for 32-bit ARM.")
    set(ARCH_FLAGS "-O3" "-mfpu=neon" "-march=armv7-a")
else()
    message(STATUS "Configuring for x86_64.")
    set(ARCH_FLAGS "-O3" "-mmmx" "-msse" "-msse2" "-msse3" "-mssse3")
endif()

# Apply all flags to our library target, using correct generator expression syntax
target_compile_options(${PROJECT_NAME} PRIVATE
    ${BASE_CXX_FLAGS}
    ${ARCH_FLAGS}
    "$<IF:$<CONFIG:Release>,-fsee;-fomit-frame-pointer;-fno-signed-zeros;-fno-math-errno;-funroll-loops,\"\">"
)

# --- Set Include Directories per Target ---
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<TARGET_PROPERTY:sophus,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:eigen,INTERFACE_INCLUDE_DIRECTORIES>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

# --- Link Libraries per Target ---
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        sophus
        eigen
    PRIVATE
        ${OpenCV_LIBS}
)


# --- Handle ROS / Catkin (if used) ---
option(USE_ROS "Set to ON to build with Catkin" OFF)
if(USE_ROS)
    find_package(catkin REQUIRED COMPONENTS roscpp cmake_modules)
    target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
    catkin_package(
        DEPENDS Eigen OpenCV Sophus
        CATKIN_DEPENDS roscpp
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
    )
endif()


# --- Handle Installation and non-ROS Package Config ---
if(NOT USE_ROS)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

    set(vikit_common_LIBRARIES ${PROJECT_NAME})
    set(vikit_common_LIBRARY_DIR ${CMAKE_BINARY_DIR})
    set(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

    # This creates the vikit_commonConfig.cmake file
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake
        @ONLY IMMEDIATE
    )
    export(PACKAGE vikit_common)

    # Install the headers and library file
    install(DIRECTORY include/vikit DESTINATION include FILES_MATCHING PATTERN "*.h")
    install(TARGETS ${PROJECT_NAME} DESTINATION lib)
endif()