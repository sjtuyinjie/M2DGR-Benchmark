# sr_livo

## Compile

```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone sr_livo_M2DGRP
cd ../..
catkin_make
```
## Run M2DGR example

```
source devel/setup.bash

roslaunch sr_livo livo_m2dgr.launch

rosbag play door_02.bag
```

## Run M2DGR-plus example
```
source devel/setup.bash
roslaunch sr_livo livo_m2dgr_plus.launch

# 开启imu坐标转换
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

# 开启lidar格式转换
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play anomaly.bag --topic /rslidar_points /camera/color/image_raw /camera/imu

```


