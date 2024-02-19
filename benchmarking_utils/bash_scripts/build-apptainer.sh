#!/usr/bin/env bash

ws_path="$(dirname "$(realpath "$0")")"/../../../..
container_path="${HOME}"/benchmarking

if [[ ! -d "${container_path}" ]]; then
    apptainer build \
        --bind "${ws_path}":/opt/catkin_ws \
        --sandbox "${container_path}" \
        "${ws_path}"/src/benchmarking_planners/benchmarking_utils/apptainer/benchmarking.def
fi

apptainer exec \
    --writable "${container_path}" \
    "${ws_path}"/src/benchmarking_planners/benchmarking_utils/bash_scripts/build-ws.sh

if [[ ! -f "${ws_path}"/src/benchmarking_planners/robowflex_resources/fetch/robots/fetch.backup ]]; then
    apptainer exec \
        --writable "${container_path}" \
        "${ws_path}"/src/benchmarking_planners/benchmarking_utils/bash_scripts/post-build.sh
fi