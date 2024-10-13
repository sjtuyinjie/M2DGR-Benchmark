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
![aloam_door](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/A_LOAM_M2DGRP/image/2024-10-12%2011-48-31%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

## run M2DGR-plus example
```
    source devel/setup.bash
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    rosbag play tree3.bag /rslidar_points:=/velodyne_points
```
![aloam_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/A_LOAM_M2DGRP/image/2024-10-09%2021-06-38%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

