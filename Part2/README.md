# ROS Project Part 2

ROS + Gazebo project about a Differential Drive robot. The project is split in two parts. The second part contains all that's needed to perform SLAM for a Differential Drive Robot using ROS. The bags used for testing are not provided. In the `odometry_node` folder we provide the node performing odometry computations from IMU or encoders data. The `mapping_and_localization` folder contains the files needed to perform mapping and localization.

## Instructions
-To switch between the IMU and Encoders odometry mode, modify `odom_source` to either `imu` or `encoders` using the command 
```rosparam set odom_source <source>```
The default initialization value is `encoders`, specified in the `/cfg/source_param.yml` file.

- To test the odometry node on different bags, modify the file `launcher/launcher.launch` commenting/uncommenting the various args. For the integration, we mixed Range-Kutta (for small values) and the exact method. The contribution of the IMU is only on the angular velocity. The bags the project was developed for contained some discontinuous points (noise at capture time), to remove them I filtered out data preventing continuous motion. 

-To build a map, launch `gmapping` use the `gmapping_launcher.launch` file. To modify the bag used for the mapping, modify the line:
```<node pkg="rosbag" type="play" name="player" output="screen" args="--clock $(find mapping_and_localization)/bags/corridor_long.bag"/>```

-To perform localization, use the file `amcl_launcher.launch`. To modify the bag used for the localization, modify the line:
```<node pkg="rosbag" type="play" name="player" output="screen" args="--clock $(find mapping_and_localization)/bags/corridor_short.bag"/>```
-To modify the map used at localization time, modify the line:
```<arg name="map_file" default="$(find mapping_and_localization)/maps/imu_map_ended_bag.yaml"/>```



