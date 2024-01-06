#!/usr/bin/env bash

cd "${0%/*}/../.." && \
    if [[ "$(uname -s)" == "Darwin" ]]; then
        docker build -t benchmarking -f benchmarking_utils/docker/Dockerfile .
    else
        sudo docker build -t benchmarking -f benchmarking_utils/docker/Dockerfile .
    fi
