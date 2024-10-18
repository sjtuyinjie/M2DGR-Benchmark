# vins_momo

## Compile

```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone vins_momo_M2DGRP
cd ../..
catkin_make
```
## Run M2DGR example

```
source devel/setup.bash
roslaunch vins_estimator m2dgr.launch

source devel/setup.bash
roslaunch vins_estimator vins_rviz.launch

rosbag play door_02.bag
```
![vinsmomo_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/vins_momo_M2DGRP/image/Peek%202024-10-17%2023-00.gif)
## Run M2DGR-plus example
```
source devel/setup.bash
roslaunch vins_estimator m2dgr_plus.launch 
    
source devel/setup.bash
roslaunch vins_estimator vins_rviz.launch

rosbag play tree3.bag --topic /camera/color/image_raw /camera/imu 
```
![vinsmomo_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/vins_momo_M2DGRP/image/plus.gif)

