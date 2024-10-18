# A-LOAM

A-Loam only needs to change the lidar topic as `/velodyne_point` on MRDGR and M2DGR-Plus. 

## Compile
Please see the [link](https://github.com/HKUST-Aerial-Robotics/A-LOAM) for code and compilation.

## run M2DGR example
```
    source devel/setup.bash
    roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch
    rosbag play door_02.bag 
```
![aloam_door](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/A_LOAM_M2DGRP/image/Peek%202024-10-13%2012-10.gif)

## run M2DGR-plus example
```
    source devel/setup.bash
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    rosbag play tree3.bag /rslidar_points:=/velodyne_points
```
![aloam_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/A_LOAM_M2DGRP/image/plus.gif)


