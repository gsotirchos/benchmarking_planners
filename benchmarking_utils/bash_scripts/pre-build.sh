#!/usr/bin/env bash

ws_path="$(dirname "$0")"/../../../..
cd "${ws_path}" || exit

# Install generic packages
apt-get update
apt-get install -y \
    build-essential pkg-config python3-catkin-tools rospack-tools \
    cmake wget git zip python3-pip

# Set up GCC 7 (for compiling SMPL)
apt-get install -y gcc-7 g++-7
export CC="$(type -p gcc-7)"
export CXX="$(type -p g++-7)"
export CMAKE_GENERATOR=

# Install SBPL (SMPL dependency)
cd "${ws_path}"/.. || exit
git clone -b epase https://github.com/shohinm/sbpl
mkdir -p sbpl/build
cd sbpl/build || exit
cmake -Wno-dev .. \
    && make \
    && make install

# Install urdfpy for sbpl2urdf
pip install --upgrade pip
pip install urdfpy numpy==1.20

# Install workspace dependencies
cd "${ws_path}" || exit
rosdep install --from-paths ./src -i -r -y
