cmake_minimum_required(VERSION 3.8)
project(fast)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# --- Set base warning flags ---
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

# --- Define all source and header files ---
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

# --- Append architecture-specific sources and compiler flags (Old Style) ---
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|armv8")
  # For 64-bit ARM platforms like your Jetson Xavier AGX
  message(STATUS "Configuring for 64-bit ARM (aarch64/armv8).")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
  list(APPEND SOURCES src/faster_corner_9_neon.cpp)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  # For 32-bit ARM platforms
  message(STATUS "Configuring for 32-bit ARM (e.g., armv7-a).")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mfpu=neon -march=armv7-a")
  list(APPEND SOURCES src/faster_corner_9_neon.cpp)

else()
  # For x86/x86_64 platforms
  message(STATUS "Configuring for x86/x86_64 architecture.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mmmx -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")
  list(APPEND SOURCES src/faster_corner_10_sse.cpp)
endif()

# --- Create the library target ---
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# --- Set public include directories for this library ---
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

################################################################################
# --- Installation Rules ---
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)