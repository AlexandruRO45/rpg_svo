cmake_minimum_required(VERSION 3.8)
project(fast)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "-Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -O3 -fomit-frame-pointer -funsafe-math-optimizations")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    # AArch64 (64-bit ARM)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+simd")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    # ARMv7 (32-bit ARM)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon")
else()
    # x86/x86_64
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse2 -msse3 -mssse3")
endif()

# find_package(ament_cmake REQUIRED)

set(HEADERS
  include/fast/corner_9.h
  include/fast/corner_10.h
  include/fast/fast.h
  include/fast/faster_corner_utilities.h
  )

set(SOURCES
  src/fast_9.cpp
  src/fast_10.cpp
  src/fast_10_score.cpp
  src/nonmax_3x3.cpp
  )

# TODO: FIND OUT THE Neon issue https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
#   LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_9_neon.cpp)
# ENDIF()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_10_sse.cpp)
ENDIF()

add_library(${PROJECT_NAME} SHARED ${SOURCES})
# target_compile_options(${PROJECT_NAME} PRIVATE
#   -Wall
#   -Werror
#   -Wno-unused-variable
#   -Wno-unused-but-set-variable
#   -Wno-unknown-pragmas
#   -O3
#   -fomit-frame-pointer
#   -funsafe-math-optimizations 
# )
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

# ament_export_libraries(${PROJECT_NAME})
# ament_export_include_directories(include)
# ament_package()

