FROM ros:noetic-ros-base
MAINTAINER George Sotirchos G.Sotirchos@student.tudelft.nl

# Install generic packages
RUN apt-get update
RUN apt-get install -y \
        build-essential pkg-config python3-catkin-tools \
        cmake wget git zip python3-pip

# Install SBPL (SMPL dependency)
RUN git clone -b epase https://github.com/shohinm/sbpl
RUN mkdir -p sbpl/build
WORKDIR /sbpl/build
RUN cmake -Wno-dev .. \
    && make \
    && make install
WORKDIR /

# Set up GCC 7 (for compiling SMPL)
RUN apt install -y gcc-7 g++-7
ENV CC=/usr/bin/gcc-7
ENV CXX=/usr/bin/g++-7
ENV CMAKE_GENERATOR=

# Install Pyre's dependencies
# RUN apt-get update
# RUN apt-get install -y \
#         libboost-all-dev libeigen3-dev libtinyxml2-dev libyaml-cpp-dev libhdf5-dev \
#         ros-${ROS_DISTRO}-moveit

# Install urdfpy for sbpl2urdf
RUN pip install --upgrade pip
RUN pip install urdfpy

# Setup Catkin workspace
RUN mkdir -p ws/src
WORKDIR /ws/src

# Clone SMPL (and leatherman utils)
RUN git clone --depth 1 -b benchmarking https://github.com/gsotirchos/smpl
RUN git clone --depth 1 -b base-u18 https://github.com/shohinm/leatherman

# Clone pyre
RUN git clone --depth 1 https://github.com/KavrakiLab/pyre

# Clone robowflex_resources
RUN git clone --depth 1 https://github.com/KavrakiLab/robowflex_resources

# Clone only robowflex_library and robowflex_ompl (instead of all of robowflex)
WORKDIR /ws
RUN mkdir -p repos
RUN git clone --depth 1 https://github.com/KavrakiLab/robowflex ./repos/robowflex
RUN ln -s ../repos/robowflex/robowflex_library src/robowflex_library 
RUN ln -s ../repos/robowflex/robowflex_ompl src/robowflex_ompl

# Install dependencies
RUN rosdep install --from-paths ./src -i -r -y

# Build everything
RUN catkin config \
        --extend /opt/ros/"${ROS_DISTRO}" \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            -Wno-dev
RUN catkin build -j $(($(nproc) / 2)) \
        --limit-status-rate 0.001 \
        --no-notify
RUN . devel/setup.bash

# Extract Pyre's datasets
RUN unzip "$(rospack find pyre)/datasets.zip" -d "$(rospack find pyre)"

# Prepare Fetch's urdf spheres collision model
RUN roscp robowflex_resources \
    fetch.urdf \
    "$(rospack find robowflex_resources)/fetch/robots/fetch.backup"
RUN rosrun benchmarking_utils sbpl2urdf \
    "$(rospack find sbpl_collision_checking_test)/config/collision_model_fetch.yaml" \
    "$(rospack find robowflex_resources)/fetch/robots/fetch.urdf"

ENTRYPOINT . /ws/devel/setup.bash && /bin/bash
