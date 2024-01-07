# Overview

`TODO`
 - *Brief description*
 - *Links to paper/media*
 - *Citation*


## 1. Installation

Instructions for both a [Dockerfile](benchmarking_utils/docker/Dockerfile) ([1.1.](#11-docker)) and as well as a native ROS workspace installation are provided ([1.2.](#12-native)) below. If you are not familiar with the ROS infrastructure, using the Docker installation is recommended.


### 1.1. Docker

 1. You can install docker (if not already installed) on you machine by following the instructions [here](https://docs.docker.com/get-docker/).

 2. Then clone this repository:

    `TODO` *replace SSH with HTTPS url when making public*
    ``` bash
    git clone --recurse-submodules git@github.com:gsotirchos/benchmarking_planners.git
    ```

 3. Call the image building script:

    ``` bash
    source benchmarking_planners/benchmarking_utils/docker/build-docker.sh
    ```


Now you can start an interactive docker session and follow the instructions from step [2.](#2-benchmarking) onwards. The name of the workspace is `/ws`.

```
sudo docker run --rm -it --name benchmarking_container benchmarking
```

### 1.2. Native

The following instructions have been tested on **Ubuntu 20.04**. Similar instructions should work for other Linux distributions.

 1. Install dependencies

    - Required packages:

      ``` bash
      sudo apt update
      sudo apt install \
          build-essential pkg-config \
          python3-catkin-tools rospack-tools \
          cmake wget git zip python3-pip
      ```

    - **GCC 7** (for compiling SMPL):

      ``` bash
      sudo apt install gcc-7 g++-7
      export CC="$(which gcc-7)"
      export CXX="$(which g++-7)"
      unset CMAKE_GENERATOR  # use makefiles
      ```

    - **[SBPL](https://github.com/shohinm/sbpl/tree/epase)**:

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

    - **[urdfpy](https://github.com/mmatl/urdfpy)**:

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

Source the benchmarking script to generate the planning results in the following structure `~/.ros/smpl_benchmarks/<problem-name>/<index>.log` (**note**: this will overwrite the folder's contents).

``` bash
source benchmarking_utils/bash_scripts/benchmark_smpl.sh
```


### 2.2. Benchmarking Pyre

Follow the steps 1 and 2 from the [benchmarking and visualizing section](https://github.com/KavrakiLab/pyre/tree/master#4-benchmarking-and-visualizing-the-results) of Pyre's README:
 1. Start a rosmaster instance:

    ``` bash
    roscore  # in a separate terminal
    ```

 2. Run one of the following scripts.
    - Benchmark SPARK and FLAME with full databases (500):

      ``` bash
      rosrun pyre benchmark.sh
      ```

    - Benchmark SPARK and FLAME with incremental databases (10, 30, 50, 100, 300, 500) for 'shelf_height_rot'.

      ``` bash
      rosrun pyre benchmark_inc.sh
      ```


## 3. Visualizing the results

### 3.1. Visualizing SMPL

To plot the results for SMPL launch the visualization script by providing the path to the directory containing all results for all the variants of a planning problem, e.g.:

``` bash
"$(rospack find benchmarking_utils)/scripts/visualize_smpl" ~/.ros/smpl_benchmarks/shelf_zero_test
```


### 3.2. Visualizing Pyre

Follow step 3 from the [benchmarking and visualizing section](https://github.com/KavrakiLab/pyre/tree/master#4-benchmarking-and-visualizing-the-results) of Pyre's README:

First use the `ompl_benchmark_statistics.py` script to aggregate the benchmarking results for each dataset.

``` bash
#Go to the benchmarking folder
roscd pyre/benchmark
#Call the ompl script to aggregate the results in an SQL database
python3 ompl_benchmark_statistics.py shelf_zero_test/*.log -d shelf_zero_test_results.db
python3 ompl_benchmark_statistics.py shelf_height_test/*.log -d shelf_height_test_results.db
python3 ompl_benchmark_statistics.py shelf_height_rot_test/*.log -d shelf_height_rot_test_results.db
```

You can load these files in [Planner Arena](http://plannerarena.org/) to plot the results.

If you are using the docker image you can copy the results to your host machine with:

``` bash
docker cp benchmarking_container:/ws/src/benchmarking_planners/pyre/benchmark/shelf_zero_test_results.db ./
docker cp benchmarking_container:/ws/src/benchmarking_planners/pyre/benchmark/shelf_height_test_results.db ./
docker cp benchmarking_container:/ws/src/benchmarking_planners/pyre/benchmark/shelf_height_rot_test_results.db ./
```


## Links

`TODO`
 - *link to SMPL*
 - *link to Pyre*
 - *link to …*
