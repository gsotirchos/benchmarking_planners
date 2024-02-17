FROM ros:noetic-ros-base
MAINTAINER George Sotirchos G.Sotirchos@student.tudelft.nl

SHELL ["/bin/bash", "-c"]
ARG WS_PATH="/ws"

# Setup Catkin workspace
ADD ./ "${WS_PATH}"/src/benchmarking_planners

Install dependencies
RUN "${WS_PATH}"/src/benchmarking_planners/benchmarking_utils/bash_scripts/pre-build.sh

# Build the workspace
RUN "${WS_PATH}"/src/benchmarking_planners/benchmarking_utils/bash_scripts/build-ws.sh

# Prepare for benchmarking
RUN "${WS_PATH}"/src/benchmarking_planners/benchmarking_utils/bash_scripts/post-build.sh

ENTRYPOINT source devel/setup.bash && exec /bin/bash
