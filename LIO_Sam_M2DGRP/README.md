# [LIO_SAM](https://github.com/TixiaoShan/LIO-SAM)
This file runs LIO-SAM on M2DGR and M2DGR-plus.
## 1.Dependency
### 1.1 Ubuntu and ROS
Tested with Ubuntu 20.04 and ROS Noetic.[ROS Installation](https://wiki.ros.org/ROS/Installation).
### 1.2 [GTSAM](https://gtsam.org/get_started/)(Georgia Tech Smoothing and Mapping library)
Tested with gtsam 4.0.3.
```
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
### 1.3 Livox SDK
Because need to install livox_ros_driver, install the livox SDK first.
```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install
```
## 2.Compile

```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone LIO_Sam_M2DGRP
cd ../..
catkin_make
```

## 3.Run M2DGR example

```
source devel/setup.bash

roslaunch lio_sam m2dgr_run.launch

rosbag play door_02.bag
```
![lio_sam_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LIO_Sam_M2DGRP/image/Peek%202024-10-13%2013-57.gif)
## 4.Run M2DGR-plus example

```
source devel/setup.bash

roslaunch lio_sam m2dgrplus_run.launch

rosbag play tree3.bag --topic /rslidar_points /camera/imu
```
![lio_sam_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LIO_Sam_M2DGRP/image/plus.gif)
