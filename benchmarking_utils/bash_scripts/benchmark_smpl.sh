#!/usr/bin/env bash

prefix_dset="$(rospack find pyre)/datasets/"
planners=(
    "arastar"
    # "awastar"
    # "mhastar"
    # "larastar"
    # "egwastar"
    # "padastar"
)
datasets=(
    "shelf_zero_test"
    # "shelf_height_test"
    # "shelf_height_rot_test"
)
start=1
end=100

for planner in "${planners[@]}"; do
    for dset in "${datasets[@]}"; do
        dataset="${prefix_dset}/${dset}"

        roslaunch smpl_test goal_fetch_benchmarking.launch \
            start:="${start}" \
            end:="${end}" \
            dataset:="${dataset}" \
            planner:="${planner}"
    done
done
