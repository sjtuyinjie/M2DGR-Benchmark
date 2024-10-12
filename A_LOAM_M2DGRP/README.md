# A-LOAM

A-LOAM在MRDGR和M2DGR—plus上只需要更改yaml文件即可。  

## run M2DGR-plus
在加入正确的yaml文件编译后
```
    source devel/setup.bash
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    rosbag play xx.bag /rslidar_points:=/velodyne_point
```
![aloam_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/A_LOAM_M2DGRP/image/2024-10-09%2021-06-38%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

