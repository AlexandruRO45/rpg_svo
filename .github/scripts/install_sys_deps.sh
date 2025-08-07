#!/bin/bash
set -e
ls -l /etc/yum.repos.d/ #Debug

echo "Updating repository configurations for EOL Linux..."
sed -i 's/mirrorlist/#mirrorlist/g' /etc/yum.repos.d/*.repo
sed -i 's|#baseurl=http://mirror.centos.org|baseurl=http://vault.centos.org|g' /etc/yum.repos.d/*.repo
echo "Repository configurations updated."

yum update -y
yum install -y epel-release

echo "Updating EPEL repository for EOL Linux..."
sed -i 's|metalink=|#metalink=|g' /etc/yum.repos.d/epel*.repo
sed -i 's|#baseurl=https://download.fedoraproject.org/pub|baseurl=https://archives.fedoraproject.org/pub|g' /etc/yum.repos.d/epel*.repo
echo "EPEL repository updated."


yum install -y \
    boost-devel \
    opencv-devel \
    eigen3-devel \
    suitesparse-devel \
    cmake \
    gcc-c++ \
    make
