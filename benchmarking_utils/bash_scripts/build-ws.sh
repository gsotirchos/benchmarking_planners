#!/usr/bin/env bash

cd "$(dirname "$0")"/../../../.. || exit

# Build everything
catkin config \
    --extend /opt/ros/"${ROS_DISTRO}" \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -Wno-dev
catkin build -j $(($(nproc) / 2 - 1)) \
    --limit-status-rate 0.001 \
    --no-notify
