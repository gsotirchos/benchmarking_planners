#!/usr/bin/env bash

cd "$(dirname "$0")"/../.. || exit

if [[ "$(uname -s)" == "Darwin" ]]; then
    docker build -t benchmarking2 -f benchmarking_utils/docker/Dockerfile .
else
    sudo docker build -t benchmarking2 -f benchmarking_utils/docker/Dockerfile .
fi
