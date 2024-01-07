FROM ros:noetic-ros-base
MAINTAINER George Sotirchos G.Sotirchos@student.tudelft.nl

# Install generic packages
RUN apt-get update
RUN apt-get install -y \
        build-essential pkg-config python3-catkin-tools rospack-tools \
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

# Install urdfpy for sbpl2urdf
RUN pip install --upgrade pip
RUN pip install urdfpy numpy==1.20

# Setup Catkin workspace
ADD . /ws/src/benchmarking_planners

# Install dependencies
WORKDIR /ws
RUN rosdep install --from-paths ./src -i -r -y

# Build everything
RUN catkin config \
        --extend /opt/ros/"${ROS_DISTRO}" \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            -Wno-dev
RUN catkin build -j $(($(nproc) / 2 - 1)) \
        --limit-status-rate 0.001 \
        --no-notify

# Extract Pyre's pre-generated datasets and database
RUN unzip src/benchmarking_planners/pyre/datasets.zip -d src/benchmarking_planners/pyre
RUN unzip src/benchmarking_planners/pyre/database.zip -d src/benchmarking_planners/pyre

# Update the Fetch robot's collision geometry
RUN cp src/benchmarking_planners/robowflex_resources/fetch/robots/fetch.urdf \
        src/benchmarking_planners/robowflex_resources/fetch/robots/fetch.backup
RUN /bin/bash -c "source devel/setup.bash \
        && src/benchmarking_planners/benchmarking_utils/scripts/sbpl2urdf \
            src/benchmarking_planners/smpl/sbpl_collision_checking_test/config/collision_model_fetch.yaml \
            src/benchmarking_planners/robowflex_resources/fetch/robots/fetch.urdf"

ENTRYPOINT /bin/bash -c "source devel/setup.bash && exec /bin/bash"
