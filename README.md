# easynav_outdoor_testcase

# VFF-based outdoor navigation system

This repository contains the **VFF-based outdoor navigation system** for robots using **GPS input**.

To test the system, use the following simulation repository: https://github.com/EasyNavigation/easynav_playground_summit.git

Running the Simulation:

1. Launch the simulation environment using the `easynav_playground_summit` repo.

2. In a new terminal, run the navigation system with:

```
   ros2 run easynav_system system_main \
     --ros-args \
     --params-file <your_workspace>/src/easynav_outdoor_stack/robots_params/summit_sim.params.yaml
```

3. Send a Goal from the `2D Goal Pose` button on RViz2


# Outdoor testcase

# Install

Prepare your thirparty repos:

```
cd <easynav_ws>/src
vcs import < easynav_outdoor_testcase/thirdparty.repos
```

You can now build the package.

# Run

With the same simulation, you can create a pointcloud map using for example `lidar_slam`. 

In a new terminal, run the maps builder system with for pointcloud:

```
ros2 run easynav_pointcloud_maps_builder pointcloud_maps_builder_main  --ros-args \
--params-file <your_workspace>/src/easynav_outdoor_testcase/robots_params/maps_builder.params.yaml
```


In a new terminal, run the maps builder system with for gridmap:

```
ros2 run easynav_gridmap_maps_builder gridmap_maps_builder_main  --ros-args \
--params-file <your_workspace>/src/easynav_outdoor_testcase/robots_params/maps_builder.params.yaml
```