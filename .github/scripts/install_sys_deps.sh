#!/bin/bash
set -e

yum update -y
yum install -y epel-release
yum install -y \
    boost-devel \
    opencv-devel \
    eigen3-devel \
    suitesparse-devel
