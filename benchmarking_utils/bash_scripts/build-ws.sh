#!/usr/bin/env bash

cd "$(dirname "$0")"/../../../.. || exit

export CC="$(type -p gcc-7)"
export CXX="$(type -p g++-7)"

# Build everything
catkin config \
    --extend /opt/ros/"${ROS_DISTRO}" \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -Wno-dev \
        -Wno-return-type
catkin build \
    --jobs $(($(nproc) / 2 - 1)) \
    --limit-status-rate 0.001 \
    --no-notify #\
    #--no-deps sbpl_collision_checking
