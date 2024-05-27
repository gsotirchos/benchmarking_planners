#!/usr/bin/env bash

prefix_dset="$(rospack find pyre)/datasets/"
datasets=(
    "shelf_zero_test"
    "shelf_height_test"
    "shelf_height_rot_test"
)

planners=(
    "arastar"
    #"awastar"
    #"mhastar"
    #"larastar"
    #"egwastar"
    #"padastar"
)

start=1
end=1

for planner in "${planners[@]}"; do
    for dset in "${datasets[@]}"; do
        dataset="${prefix_dset}/${dset}"

        roslaunch --wait smpl_test goal_fetch_benchmarking.launch \
            start:="${start}" \
            end:="${end}" \
            dataset:="${dataset}" \
            planner:="${planner}" &
    done
    
    wait
done

