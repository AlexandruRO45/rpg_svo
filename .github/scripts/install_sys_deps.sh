#!/bin/bash
# This script installs dependencies for the build, including building
# a specific version of OpenCV from source to ensure compatibility.

set -e
set -o pipefail

echo "--- Running yum clean and installing base dependencies on AlmaLinux... ---"
yum clean all
yum update -y

# Enable the EPEL and PowerTools/CRB repositories for build dependencies
yum install -y epel-release
yum config-manager --set-enabled crb

# Install dependencies required to build OpenCV and for the main project
yum install -y \
    boost-devel \
    eigen3-devel \
    suitesparse-devel \
    cmake \
    gcc-c++ \
    make \
    git \
    wget \
    unzip \
    yasm \
    pkg-config \
    libjpeg-turbo-devel \
    libpng-devel \
    libtiff-devel \
    libavc1394-devel \
    tbb-devel

# --- OpenCV Build from Source ---
OPENCV_VERSION="4.8.0"
echo "--- Building OpenCV version ${OPENCV_VERSION} from source... ---"

# Create a temporary directory for the build
mkdir -p /tmp/opencv_build
cd /tmp/opencv_build

# Download the source code for OpenCV and OpenCV-contrib
wget -q -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip
wget -q -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip
unzip -q opencv.zip
unzip -q opencv_contrib.zip

# Create a build directory
cd opencv-${OPENCV_VERSION}
mkdir -p build && cd build

# Configure CMake for the OpenCV build
# We disable tests and examples to speed up the build process.
cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_build/opencv_contrib-${OPENCV_VERSION}/modules \
    -D WITH_TBB=ON \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    ..

# Compile and install OpenCV
make -j$(nproc)
make install

# Update the shared library cache
ldconfig

# Clean up the build files to save space
cd /
rm -rf /tmp/opencv_build

echo "--- All dependencies, including OpenCV ${OPENCV_VERSION}, installed successfully. ---"
