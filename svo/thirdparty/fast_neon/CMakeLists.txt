project(fast)
cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")
endif()

set(SOURCES
  src/fast_9.cpp
  src/fast_10.cpp
  src/fast_10_score.cpp
  src/nonmax_3x3.cpp
)

if(DEFINED ENV{ARM_ARCHITECTURE})
  list(APPEND SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/faster_corner_9_neon.cpp)
else()
  list(APPEND SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/faster_corner_10_sse.cpp)
endif()

add_library(fast ${SOURCES})

target_include_directories(fast PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)