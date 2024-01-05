#!/usr/bin/env bash

prefix_dset="$(rospack find pyre)/datasets/"
datasets=("shelf_zero" "shelf_height" "shelf_height_rot")
start=1
end=100

for dset in "${datasets[@]}"; do
    dataset="${prefix_dset}/${dset}"

    roslaunch smpl_test goal_fetch_benchmarking.launch start:="${start}" end:="${end}" dataset:="${dataset}"
done
