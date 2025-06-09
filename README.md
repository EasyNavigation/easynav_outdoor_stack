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


# Outdoor maps builder (WIP)

This repository contains the **Maps builder for outdoor navigation**

# Install

Prepare your thirparty repos:

```
cd <easynav_ws>/src
vcs import < easynav_outdoor_stack/easynav_outdoor_maps_builder/thirdparty.repos
```

You can now build the package.

# Run

With the same simulation, you can create a pointcloud map using for example `lidar_slam`. 

In a new terminal, run the navigation system with:

```
ros2 run easynav_outdoor_maps_builder outdoor_maps_builder_main  --ros-args \
--params-file <your_workspace>/src/easynav_outdoor_stack/robots_params/summit_sim_slam.params.yaml
```

In map types, it is possible to select pcl (pointcloud) and/or gridmap.
