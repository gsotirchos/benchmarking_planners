# Overview

`TODO`
<!--
 - *Brief description*
 - *Links to paper/media*
 - *Citation*
-->


## 1. Installation

The following instructions are intended for a native ROS workspace installation, from [section 1.2](#12-native) onwards. In addition, instructions for setting up a docker container (in which the same steps can be followed) using a [Dockerfile](benchmarking_utils/docker/Dockerfile) are provided in ([section 1.1](#11-docker)) along with the additional final steps for exporting the experiments' results, in [section 3](#3-visualizing-the-results). If you are not familiar with the ROS infrastructure, using the Docker installation is recommended.


### 1.1. Docker

 1. You can install docker (if not already installed) on you machine by following the instructions [here](https://docs.docker.com/get-docker/).

 2. Then recursively clone this repository:

    ``` bash
    git clone --recurse-submodules https://github.com/gsotirchos/benchmarking_planners
    ```

 3. Call the image building script:

    ``` bash
    benchmarking_planners/benchmarking_utils/docker/build-docker.sh
    ```


Now you can start an interactive docker session and follow the instructions from [step 2](#2-benchmarking) onwards. The name of the workspace is `/ws`.

```
sudo docker run -it --name benchmarking_container benchmarking
```

### 1.2. Native

The following instructions have been tested on **Ubuntu 20.04**. Similar instructions should work for other Linux distributions.

 1. Install dependencies:

    - Required packages

      ``` bash
      sudo apt update
      sudo apt install \
          build-essential pkg-config \
          python3-catkin-tools rospack-tools \
          cmake wget git zip python3-pip
      ```

    - **GCC 7** (for building SMPL)

      ``` bash
      sudo apt install gcc-7 g++-7
      export CC="$(which gcc-7)"
      export CXX="$(which g++-7)"
      unset CMAKE_GENERATOR  # use makefiles
      ```

    - **[SBPL](https://github.com/shohinm/sbpl/tree/epase)**

      ``` bash
      git clone -b epase https://github.com/shohinm/sbpl \
          && cd sbpl
      mkdir build
      cd build
      cmake -Wno-dev .. \
          && make \
          && sudo make install
      cd ../..
      ```

    - **[urdfpy](https://github.com/mmatl/urdfpy)**

      ``` bash
      pip install urdfpy numpy==1.20
      ```


 2. Prepare a workspace and clone this repository:
    ``` bash
    mkdir benchmarking_ws/src
    cd benchmarking_ws/src
    git clone --recurse-submodule https://github.com/gsotirchos/benchmarking_planners
    ```
 3. Install ROS dependencies:
    ``` bash
    cd ..
    rosdep install --from-paths ./src -i -r -y
    ```

 4. Build the workspace:
    ``` bash
    catkin config \
        --extend /opt/ros/"${ROS_DISTRO}" \
        --cmake-args
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            -Wno-dev
    catkin build

    source devel/setup.bash
    ```

 5. Extract Pyre's pre-generated datasets and database:
    ``` bash
    unzip "$(rospack find pyre)/datasets.zip" -d "$(rospack find pyre)"
    unzip "$(rospack find pyre)/database.zip" -d "$(rospack find pyre)"
    ```

 6. Update the Fetch robot's URDF geometry using SBPL's collision 
    spheres definition:
    ``` bash
    cp "$(rospack find robowflex_resources)/fetch/robots/fetch.urdf" \
        "$(rospack find robowflex_resources)/fetch/robots/fetch.backup"
    "$(rospack find benchmarking_utils)/scripts/sbpl2urdf" \
        "$(rospack find sbpl_collision_checking_test)/config/collision_model_fetch.yaml" \
        "$(rospack find robowflex_resources)/fetch/robots/fetch.urdf"
    ```


## 2. Benchmarking

First source the workspace's environment setup file:

``` bash
source devel/setup.bash
```



### 2.1. Benchmarking SMPL

Source the benchmarking script to generate the planning results in the following pattern `~/.ros/smpl_benchmarks/<problem-name>.csv`:
<!--
> [!WARNING]
> This will overwrite the folder's contents.
-->

``` bash
source benchmarking_utils/bash_scripts/benchmark_smpl.sh
```


### 2.2. Benchmarking Pyre

By following **steps 1 and 2** from section [4) Benchmarking and Visualizing the results](https://github.com/KavrakiLab/pyre/tree/master#4-benchmarking-and-visualizing-the-results) in Pyre's README:
 1. Start a rosmaster instance:

    ``` bash
    roscore  # in a separate terminal
    ```

 2. Run one of the following scripts.
    - Benchmark SPARK and FLAME with full databases (500):

      ``` bash
      rosrun pyre benchmark.sh
      ```

    - Benchmark SPARK and FLAME with incremental databases (10, 30, 50, 100, 300, 500) for 'shelf_height_rot':

      ``` bash
      rosrun pyre benchmark_inc.sh
      ```


## 3. Visualizing the results

Based on **step 3** from section [4) Benchmarking and Visualizing the results](https://github.com/KavrakiLab/pyre/tree/master#4-benchmarking-and-visualizing-the-results) in Pyre's README:

 1. Use the `ompl_benchmark_statistics.py` script to aggregate the benchmarking results for each dataset.

    ``` bash
    # Go to the benchmarking folder
    roscd pyre/benchmark

    # Call the ompl script to aggregate the results in an SQL database
    python3 ompl_benchmark_statistics.py shelf_zero_test/*.log -d shelf_zero_test_results.db
    python3 ompl_benchmark_statistics.py shelf_height_test/*.log -d shelf_height_test_results.db
    python3 ompl_benchmark_statistics.py shelf_height_rot_test/*.log -d shelf_height_rot_test_results.db
    ```

> [!IMPORTANT]
> If you ran the experiments inside a Docker container you can copy the results files to your host machine with:
> ``` bash
> docker cp benchmarking_container:/ws/src/benchmarking_planners/pyre/benchmark/shelf_zero_test_results.db ./
> docker cp benchmarking_container:/ws/src/benchmarking_planners/pyre/benchmark/shelf_height_test_results.db ./
> docker cp benchmarking_container:/ws/src/benchmarking_planners/pyre/benchmark/shelf_height_rot_test_results.db ./
> docker cp benchmarking_container:/ws/src/benchmarking_planners/smpl/... ./  # TODO
> ```

 2. Launch the visualization script to generate plot the results:

    ``` bash
    "$(rospack find benchmarking_utils)/scripts/visualize.py"
    ```

Additionally, you can load the aggregated results (*.db) files in [Planner Arena](http://plannerarena.org/) to plot the results.


## Links

 - [Pyre](https://github.com/KavrakiLab/pyre) by Kavrakilab
 - [SMPL](https://github.com/aurone/smpl) by Search-Based Planning Lab
 - [SBPL](https://github.com/sbpl/sbpl) by Search-Based Planning Lab
 - [urdfpy](https://github.com/mmatl/urdfpy) by Matthew Matl
