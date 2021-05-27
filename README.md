# Next-Best-View Exploration Planner
**license:** [CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/deed.en) <br/>
**autors:**  Respall Victor Massague, Devitt Dmitry, Fedorenko Roman

## Table of Contents
* [Overview](#overview)
* [Installation Instructions](#installation-instructions)
* [Parameters](#parameters)
  * [General Parameters](#general-parameters)
  * [System Parameters](#system-parameters)
  * [Map Parameters](#map-parameters)
  * [Gain Parameters](gain-parameters)
  * [History Graph Parameters](history-graph-parameters)
  * [Path Planning Parameters](path-planning-parameters)
  * [Scenario Parameters](scenario-parameters)
* [Paper and Video](#paper-and-video)
* [Contact](#contact)

# Overview

The Next-Best-View Exploration Planner (NBV Exploration Planner) is a real-time capable exploration path planner. 
From the current pose it expands a tree to find a next pose that gives a high exploration gain in the vicinity of the MAV.
As the vehicle is performing the exploration process, the current pose is stored as a node, indicating wheather a frontier is encountered in the vicinity, 
in a new data structure called History Graph, which creates a network of visited positions to rely on, when no gain is found in the vicinity of the MAV.

# Installation Instructions
To install NBV Exploration Planner, please install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu/) or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu/) for Ubuntu.

First install additional system dependencies (substitute DISTRO for kinetic or melodic as necessary):

```
sudo apt-get install python-wstool python-catkin-tools ros-DISTRO-cmake-modules protobuf-compiler autoconf
```

Next, add a few other dependencies. If you don’t have a catkin workspace yet, set it up as follows:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Clone the planner repository to your catkin workspace and install package dependecies

```
cd ~/catkin_ws/src/
git clone https://github.com/InnopolisAero/nbv_exploration_planner
wstool init . ./nbv_exploration_planner/dependencies.rosinstall
wstool update
```

If you have already initialized wstool replace the above `wstool init` with `wstool merge -t`

Finally, compile or build the workspace
```
cd ~/catkin_ws/src/
catkin build
```

# Parameters

## General Parameters
* `tf_frame`: The base frame used when looking up tf transforms. This is also the frame that most outputs are given in (string, default: "world")
* `wp_z`: Maximum height to perform the initialization motion (double, 1.0m)
* `speed_rotate`: Yaw speed of the initialization motion (double, 0.5rad/s)
* `shift_initial_x`: Shift of the MAV position after the initialization motion in x-direction (double, default: 0.0)
* `shift_initial_y`: Shift of the MAV position after the initialization motion in y-direction (double, default: 0.0)
* `shift_initial_z`: Shift of the MAV position after the initialization motion in z-direction (double, default: 0.0)
* `path_plot`: Path to store a log file of the exploration process, if left empty the log is not created. (string, required)

## System Parameters
* `system/v_max`: Maximal linear speed (double, default: 1.0m/s)
* `system/dyaw_max`: Maximal yaw speed (double, default: 0.5rad/s)
* `system/camera/pitch`: Pitch angle of the depth sensors (list of doubles, default: 15deg)
* `system/camera/horizontal`: Horizontal Field of View (FOV) of the depth sensors (list of doubles, default: 90deg)
* `system/camera/vertical`: Vertical Field of View (FOV) of the depth sensors (list of doubles, default: 60deg)
* `system/bbx/x`: MAV bounding box for collision avoidance, dimension in x-direction (double, default: 0.5m)
* `system/bbx/y`: MAV bounding box for collision avoidance, dimension in y-direction (double, default: 0.5m)
* `system/bbx/z`: MAV bounding box for collision avoidance, dimension in z-direction (double, default: 0.3m)
* `system/bbx/overshoot`: Extension for collision check, that is added to the nominal length of the path segment (double, default: 0.5m)

## Map Parameters
This parameters correspond to the package [Voxblox](https://github.com/ethz-asl/voxblox). 
Please refer to the [documentation](https://voxblox.readthedocs.io/en/latest/pages/The-Voxblox-Node.html#) for more information.

* `tsdf_voxel_size`: The size of the TSDF voxel, or in other words, resolution of the map (double, default: 0.15m)
* `tsdf_voxels_per_side`: TSDF voxels per side of an allocated block. Must be a power of 2 (int, default: 16) 
* `min_time_between_msgs_sec`: Minimum time to wait after integrating a message before accepting a new one (double, default: 0.0s)
* `min_ray_length_m`: The point at which the ray casting will start (double, default: 0.1m)
* `max_ray_length_m`: The maximum range out to which a ray will be cast (double, default: 5.0m)
* `voxel_carving_enabled`: If true, the entire length of a ray is integrated, if false only the region inside the trunaction distance is used. (bool, default: true)
* `color_mode`: The method that will be used for coloring the mesh. ([“color”, “height”, “normals”, “lambert”, “gray”], default: "color")
* `use_const_weight`: If true all points along a ray have equal weighting (bool, default: false)
* `publish_tsdf_map`: Whether to publish the complete TSDF map periodically over ROS topics (bool, default: false)
* `publish_esdf_map`: Whether to publish the complete ESDF map periodically over ROS topics (bool, default: false)
* `update_mesh_every_n_sec`: Rate at which the mesh topic will be published to, a value of 0 disables (double, default: 0.0s)
* `method`: Method for TSDF integration (["simple", "merged", "fast"], default: "merged")
* `allow_clear`: If true points beyond the `max_ray_length_m` will be integrated up to this distance (bool, default: true)
* `use_tf_transforms`: If true the ros tf tree will be used to get the pose of the sensor relative to the world. 
If false the pose must be given via the `transform` topic. (bool, default: true)
* `world_frame`: The base frame used when looking up tf transforms. This is also the frame that most outputs are given in (string, default: "world")
* `verbose`: Prints additional debug and timing information (bool, default: true)


## Gain Parameters
* `nbvep/gain/free`: Gain for visible free voxels (double, default: 0.0)
* `nbvep/gain/occupied`: Gain for visible occupied voxels (double, default: 0.0)
* `nbvep/gain/unmapped`: Gain for visible unmapped voxels (double, default: 1.0)
* `nbvep/gain/range`: Maximum distance of voxels to be considered for the gain computation (double, default: 1.0) 
* `nbvep/gain/zero`: Threshold for considering a node with gain or not (double, default: 0.0)
* `nbvep/gain/degressive_coeff`: Weighting parameter for the summation of the node specific gains along the branches of the tree. A high coefficient punishes distance more (double, default: 0.0)

## Path Planning Parameters
* `nbvep/tree/extension_range`: Maximum length for new tree branches when sampling (double, default: 1.0)
* `nbvep/tree/vicinity_range`: Maximum range to build the RRT around the MAV current position (double, default: 5.0m)
* `nbvep/tree/init_iterations`: Minimum number of nodes in the RRT at every iteration (int, default: 15)
* `nbvep/tree/cutoff_iterations`: Maximal number of iterations, when the exploration task is considered to be completed when no gain has been found (int, default: 200)
* `nbvep/dt`: Time step for path sampling (double, default: 0.1)

## History Graph Parameters
* `nbvep/graph/node_vicinity_range`: Maximum range to search for frontier voxels around a History Graph node (double, default: 5.0m) 
* `nbvep/graph/zero_frontier_voxels`: Minimum volume of voxels to consider a History Graph node active (double, default: 0.0)

## Scenario Parameters
* `bbx/minX`: Minimum x-coordinate value of the scenario (double, required)
* `bbx/minY`: Minimum y-coordinate value of the scenario (double, required)
* `bbx/minZ`: Minimum z-coordinate value of the scenario (double, required)
* `bbx/maxX`: Maximum x-coordinate value of the scenario (double, required)
* `bbx/maxY`: Maximum y-coordinate value of the scenario (double, required)
* `bbx/maxZ`: Maximum z-coordinate value of the scenario (double, required)

# Paper and Video
If you use this software in a scientific publication, please cite the following paper, available [here](https://www.researchgate.net/publication/339447464_Unmanned_Aerial_Vehicle_Path_Planning_for_Exploration_Mapping):

```
@article{respallunmanned,
  title={Unmanned Aerial Vehicle Path Planning for Exploration Mapping},
  author={Respall, Victor Massague and Devitt, Dmitry and Fedorenko, Roman}
}
```

Video running on-board of a real MAV can be see [here](https://youtu.be/o1RbLLVwFTA) and a simulation of the History Graph capabilities [here](https://www.youtube.com/watch?v=V9ppuyBhTFU) 

# Contact

You can contact for any question or remark:
* [Victor Massagué Respall](mailto:v.respall@innopolis.ru)
