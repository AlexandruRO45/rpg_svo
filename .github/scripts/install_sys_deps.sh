#!/bin/bash
set -e

dnf update -y
dnf install -y epel-release
dnf install -y \
    boost-devel \
    opencv-devel \
    eigen3-devel \
    suitesparse-devel
