# easynav_outdoor_testcase

# Install

First get all the required dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```

# Run
You can create a pointcloud map using for example `lidar_slam`. 

In a new terminal, run the maps builder system, for example if you are using gridmap:

```
ros2 run easynav_gridmap_maps_builder gridmap_maps_builder_main  --ros-args \
--params-file <your_workspace>/src/easynav_outdoor_testcase/robots_params/maps_builder.params.yaml
```
