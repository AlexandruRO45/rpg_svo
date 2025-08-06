#!/bin/bash
set -eux # Exit on error, print commands

echo "🚀 Installing dependencies for ${RUNNER_OS}"

if [[ "${RUNNER_OS}" == "Linux" ]]; then
  # For manylinux containers used by cibuildwheel
  apt-get update
  apt-get install -y libeigen3-dev libopencv-dev libboost-all-dev

elif [[ "${RUNNER_OS}" == "macOS" ]]; then
  # For macOS runners
  export HOMEBREW_NO_AUTO_UPDATE=1
  brew install eigen opencv boost
fi

echo "✅ Dependencies installed."