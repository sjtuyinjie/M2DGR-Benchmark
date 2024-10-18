# FAST-Livo

## Compile
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone FAST_livo_M2DGRP
cd ../..
catkin_make
```
##  Run M2DGR example
```
source devel/setup.bash
roslaunch fast_livo mapping_m2dgr.launch
rosbag play door_02.bag 
```
![fast_livo_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/FAST_livo_M2DGRP/image/Peek%202024-10-13%2013-57.gif)
##  Run M2DGR-plus example
```
source devel/setup.bash
roslaunch fast_livo mapping_m2dgr_plus.launch

# 开启imu坐标转换
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

# 开启lidar格式转换
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play anomaly.bag --topic /rslidar_points /camera/color/image_raw /camera/imu
```
![fast_livo_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/FAST_livo_M2DGRP/image/plus.gif)
