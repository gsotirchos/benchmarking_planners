#!/usr/bin/env bash

user_specified_path="$1"
container_path="${user_specified_path:-"${HOME}"/benchmarking}"
ws_path="$(dirname "$(realpath "$0")")"/../../../..

if [[ ! -f "${container_path}"/singularity ]]; then
    apptainer build \
        --bind "${ws_path}":/opt/catkin_ws \
        --sandbox "${container_path}" \
        "${ws_path}"/src/benchmarking_planners/benchmarking_utils/apptainer/benchmarking.def
else
    apptainer exec \
        --writable "${container_path}" \
        "${ws_path}"/src/benchmarking_planners/benchmarking_utils/bash_scripts/build-ws.sh

    if [[ ! -f "${ws_path}"/src/benchmarking_planners/robowflex_resources/fetch/robots/fetch.backup ]]; then
        apptainer exec \
            --writable "${container_path}" \
            "${ws_path}"/src/benchmarking_planners/benchmarking_utils/bash_scripts/post-build.sh
    fi
fi
