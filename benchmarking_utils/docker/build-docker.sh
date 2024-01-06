#!/usr/bin/env bash

cd "${0%/*}" \
    && sudo docker build -t benchmarking .
#    &&  sudo docker buildx build -t benchmarking .
