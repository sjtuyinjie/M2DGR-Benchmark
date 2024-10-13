# LINS

在M2DGR上运行LINS只需要更改相应的yaml文件和launch文件。

M2DGR-plus的激光雷达话题`/rslidar_points`不包含`ring`和`time`的信息，所以需要对该信息进行手动添加，添加方法参考[Lego-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)。

## Compile
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone LINS_M2DGRP
cd ../..
catkin_make
```

## run M2DGR example
```
source devel/setup.bash

roslaunch lins run_m2dgr.launch

rosbag play door_02.bag --clock
```
![LINS_door](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LINS_M2DGRP/image/2024-10-10%2020-20-09%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

## run M2DGR-plus example
```
source devel/setup.bash
roslaunch lins run_m2dgr_plus.launch

# 开启imu坐标转换
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

# 开启lidar格式转换
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play tree3.bag --clock --topic /rslidar_points /camera/color/image_raw /camera/imu
```
![LINS_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LINS_M2DGRP/image/2024-10-10%2020-20-09%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)
