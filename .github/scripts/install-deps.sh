#!/bin/bash
set -eux

echo "🚀 Installing C++ dependencies..."

# For Linux (inside manylinux containers)
if command -v yum &> /dev/null; then
    echo "Found YUM package manager (CentOS/manylinux)."
    yum install -y eigen3-devel opencv-devel boost-devel

elif command -v apt-get &> /dev/null; then
    echo "Found APT package manager (Debian/Ubuntu)."
    apt-get update
    apt-get install -y libeigen3-dev libopencv-dev libboost-all-dev

# For macOS (on the runner host)
elif command -v brew &> /dev/null; then
    echo "Found Homebrew package manager (macOS)."
    export HOMEBREW_NO_AUTO_UPDATE=1
    brew install eigen opencv boost

else
    echo "Error: Could not find a known package manager (yum, apt, or brew)."
    exit 1
fi

echo "✅ Dependencies installed."