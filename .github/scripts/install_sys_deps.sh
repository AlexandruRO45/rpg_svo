#!/bin/bash
set -e


echo "Updating CentOS 7 repository configurations..."
sed -i 's/mirrorlist/#mirrorlist/g' /etc/yum.repos.d/CentOS-*
sed -i 's|#baseurl=http://mirror.centos.org|baseurl=http://vault.centos.org|g' /etc/yum.repos.d/CentOS-*
echo "Repository configurations updated."

yum update -y
yum install -y epel-release

sed -i 's/mirrorlist/#mirrorlist/g' /etc/yum.repos.d/epel*.repo
sed -i 's|#baseurl=http://download.fedoraproject.org/pub|baseurl=http://archives.fedoraproject.org/pub|g' /etc/yum.repos.d/epel*.repo


yum install -y \
    boost-devel \
    opencv-devel \
    eigen3-devel \
    suitesparse-devel \
    cmake \
    gcc-c++ \
    make
