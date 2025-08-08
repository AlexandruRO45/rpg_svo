#!/bin/bash
set -e


echo "Running yum clean and installing dependencies on AlmaLinux..."
yum clean all
yum update -y


yum install -y epel-release


yum install -y \
    boost-devel \
    opencv-devel \
    eigen3-devel \
    suitesparse-devel \
    cmake \
    gcc-c++ \
    make

echo "All dependencies installed successfully."