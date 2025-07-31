# ARENA

ARENA is an adaptive risk-aware and energy-efficient navigation approach used in multi-objectives path planning problem in complex 3D environments. This algorithm enables online trajectory adaptation by optimizing multiple objectives on a Pareto front with a genetic-based algorithm and using non-uniform rational b-spline as a continuous representation of trajectories. A novel risk-aware voting algorithm ensures adaptivity by choosing one solution of the optimization according to evolving mission risks.

If you find this work useful or interesting, we kindly invite you to star :star: the library!

### *Note
As of now, the algorithm has been only applied to real UAV power line inspection scenarios. We plan to investigate the generalization of the method to different scenarios and different platforms in future works. This algorithm represents a framework of multi-objective optimisation in the field of path planning or decision making for unmanned autonomous robots. As of now, we can, but are not limited, to optimize time, safety, and energy along a trajectory for an inspection UAV.

## Related paper
*ARENA: Adaptive Risk-aware and Energy-efficient NAvigation for Multi-Objective 3D Infrastructure Inspection with a UAV*, David-Alexandre Poissant, Alexis Lussier Desbiens, François Ferland, and Louis Petit (To be submitted to ICRA2026). 

<p align="center"><strong>Real flights in a simulated power line inspection scenario</strong></p>
<p align = "center">
<img src="doc/images/real_flight_scenarios.png" width="45%" title="Real flights in a simulated power line inspection scenario">
<img src="doc/images/real_flight_scenarios_visualization.png" width="45%" title="RVIZ visualization of real flights in simulated power line inspection scenario">
</p>

In the UAV power line inspection scenario, we optimize for time, safety, and energy. We can see trajectoris in green staying further from obstacles, trajectories in pink being faster and more energy-efficient, and trajectories in white being a mix of every objective.

<p align="center"><strong>Pareto front visualization after optimization</strong></p>
<p align = "center">
<img src="doc/images/pareto_front_visualization.png" title="Pareto front visualization" width="60%">
</p>

After the genetic-based optimization, the ARENA framework proposes a Pareto front of optimized trajectories answering different multiple situations.

<p align="center"><strong>Trajectories according to different sets of mission risks for an inspection UAV</strong></p>
<p align = "center">
<img src="doc/images/trajectories_according_to_risks_construction_site.png" width="45%" title="Optimized trajectories that would have been chosen in different risks scenarios in an urban construction site">
<img src="doc/images/trajectories_according_to_risks_power_lines.png" width="45%" title="Optimized trajectories that would have been chosen in different risks scenarios on power lines">
</p>

Our voting algorithm is adapted for any amount of objectives, as long as a factor for each one comes from evolving mission risks. You can also fix objective weight as a design intent.

<p align="center"><strong>Voting algorithm schematic</strong></p>
<p align = "center">
<img src="doc/images/voting_algorithm.png" title="Pareto front visualization" width="60%">
</p>


## Acknowledgements
* This work extends [MOAR-Planner](https://github.com/SAFiRLab/moar-planner) to 3D environments and optimizes all path planning objectives as a Pareto front.


## Quick start
This library was primarly developped using docker. Therefore, for easy quick start to test the library, we suggest installing [docker](https://www.docker.com/products/docker-desktop/).

Taking for granted docker has been installed and docker engine is running:

1. Build the container image using OS specified script in [scripts](./scripts) (this can take awhile)
2. Run the container image using OS specified script in [scripts](./scripts)
3. In the container: ```colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release```

We recommend using [VSCode](https://code.visualstudio.com/) with the dev container for remote explorer [extension](https://code.visualstudio.com/docs/remote/remote-overview) since you can easily launch multiple terminals and therefore launch multiple ROS2 nodes at the same time.

4. In the container: ```ros2 launch arena_core costmap_3D_node```
5. In the container (another terminal): ```ros2 launch arena-core linedrone_test_node```
6. In the container (another terminal) send planning request informations:

```
ros2 topic pub /linedrone_test_node/planning_activation std_msgs/msg/Bool "data: true" --once
ros2 topic pub /linedrone_test_node/goal_pose geometry_msgs/msg/PointStamped "{
  header: {
    frame_id: 'map'
  },
  point: {
    x: 59.367,
    y: -26.573,
    z: 44.997
  }
}" --once
```

Note that these are examples start-goal positions!

We also note that we use [Foxglove](https://foxglove.dev/download) to visualize ros2 data with our linedrone demo. This container image already comes with foxglove bridge so if you want to be able to visualize in foxglove run:

7. In the container: ```ros2 run foxglove_bridge foxglove_bridge```


## Manual setup environment

To build your dev environment you can follow the steps in the [Dockerfile](./Dockerfile), but here is a detailed breakdown of the needed steps.

1. **ROS2 Facultative** If you want to run the linedrone demo or run the costmap node, you need a ROS2 distribution.
2. **ROS2 Facultative** Create your ROS2 workspace.
3. **ROS2 Facultative** install linedrone demo and costmap_node dependancies (replace DISTRO variable with your ROS2 distribution ex: humble):

```
DISTRO=humble
apt-get update && apt-get install -y \
    ros-${DISTRO}-ompl \
    ros-${DISTRO}-octomap \
    ros-${DISTRO}-octomap-ros \
    ros-${DISTRO}-octomap-server \
    libjsoncpp-dev \
    libsecret-1-dev \
    libccd-dev && \
    rm -rf /var/lib/apt/lists/*
```

4. Clone the ARENA repository and **ROS2 Facultative*** place the arena_core package in your worskpace/src.

The arena_core library has 3 important dependancies: , [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), [OMPL](https://ompl.kavrakilab.org/), [pagmo2](https://esa.github.io/pagmo2/), and 2 **Facultative** dependancies: [octomap](https://octomap.github.io/), and [FCL](https://github.com/flexible-collision-library/fcl). 

5. **Facultative** If you want to run the linedrone demo or the costmap_node, you will need all 5 external libraries and ROS2 so you should have done steps 1 through 4 and therefore OMPL, Octomap, and Eigen3 should already be installed.

6. You will also need to install FCL:

```
cd ~
git clone https://github.com/flexible-collision-library/fcl.git
cd ~/fcl
mkdir build
cd build
source /opt/ros/humble/setup.bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DFCL_WITH_OCTOMAP=ON -DOCTOMAP_INCLUDE_DIR=/opt/ros/humble/include -DOCTOMAP_LIBRARY=/opt/ros/humble/lib/x86_64-linux-gnu/liboctomap.so
make -j4
sudo make install
```

7. **If you don't need to run ROS2 demos** (steps 1 through 5), here's how to install Eigen3:

```
sudo apt install libeigen3-dev
```

8. **If you don't need to run ROS2 demos** (steps 1 through 5), you can follow [this documentation](https://ompl.kavrakilab.org/installation.html) to install OMPL.

9. For pagmo2, we have a temporary fork to fix some bugs and add small features in order to check initial population boundaries and give the possibility to change the initial population. **We have no intent of distributing this fork to respect their GPL-3.0 licence** (Note that we implement the fixes and features [here](https://github.com/Dave-Poissant/pagmo2/tree/work_on_archipelago) and you are free to change the git clone link). Here is how to install it from source:

```
cd ~
git clone https://github.com/esa/pagmo2.git
# If you are cloning our fork checkout the work_on_archipelago branch:
# git checkout -b work_on_archipelago origin/work_on_archipelago
git fetch
git pull
cd /home/pagmo2
mkdir build
cd build
cmake .. -DPAGMO_WITH_EIGEN3=ON -DCMAKE_BUILD_TYPE=Release
cmake --build
sudo cmake --build . --target install
```


## Code usage for different optimization problems

We give a full explanation of how to use the library's APIs [here](./arena_core/libs/README.md)


## Licence
The source code is released under the [BSD 3-Clause](https://opensource.org/license/bsd-3-clause) licence.

## Maintenance
We are still working on extending the proposed framework and improving code reliability and optimization.

For any issues or questions, please contact David-Alexandre Poissant (david-alexandre.poissant@usherbrooke.ca)

