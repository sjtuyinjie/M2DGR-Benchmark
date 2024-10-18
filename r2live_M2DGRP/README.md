# R2live 
## Compile
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone r2live_M2DGRP
cd ../..
catkin_make
```
##  Run M2DGR example
```
source devel/setup.bash
roslaunch r2live m2dgr.launch
rosbag play door_02.bag 
```
![r2live_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/r2live_M2DGRP/image/Peek%202024-10-17%2022-08.gif)
##  Run M2DGR-plus example
```
source devel/setup.bash
roslaunch r2live m2dgr_plus.launch

# 开启imu坐标转换
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

# 开启lidar格式转换
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play anomaly.bag --topic /rslidar_points /camera/color/image_raw /camera/imu

```
![r2live_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/r2live_M2DGRP/image/plus.gif)

