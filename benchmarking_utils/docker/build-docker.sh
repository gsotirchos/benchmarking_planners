#!/usr/bin/env bash

cd "${0%/*}" \
    && docker build -t benchmarking .
#    &&  docker buildx build -t benchmarking .
