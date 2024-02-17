#!/usr/bin/env bash

cd "$(dirname "$0")"/../../../.. || exit

source devel/setup.bash

# Extract Pyre's pre-generated datasets and database
unzip "$(rospack find pyre)"/datasets.zip -d "$(rospack find pyre)"
unzip "$(rospack find pyre)"/database.zip -d "$(rospack find pyre)"

# Update the Fetch robot's collision geometry
cp "$(rospack find robowflex_resources)"/fetch/robots/fetch.urdf \
    "$(rospack find robowflex_resources)"/fetch/robots/fetch.backup
"$(rospack find benchmarking_utils)"/scripts/sbpl2urdf \
    "$(rospack find sbpl_collision_checking_test)"/config/collision_model_fetch.yaml \
    "$(rospack find robowflex_resources)"/fetch/robots/fetch.urdf
