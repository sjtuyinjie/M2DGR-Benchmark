# [vins_mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
This file runs vins_mono on M2DGR and M2DGR-plus.
## 1.Dependency
### 1.1 Ubuntu and ROS
Tested with Ubuntu 20.04 and ROS Noetic.[ROS Installation](https://wiki.ros.org/ROS/Installation).
### 1.2 Ceres 
C++ library for modeling and solving large, complicated optimization problems.Tested with ceres1.14.0.
```
sudo apt-get install -y libgoogle-glog-dev
sudo apt-get install -y libatlas-base-dev
wget -O ~/Downloads/ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip
cd ~/Downloads/ && unzip ceres.zip -d ~/Downloads/
cd ~/Downloads/ceres-solver-1.14.0
mkdir ceres-bin && cd ceres-bin
cmake ..
sudo make install -j4
```
### 1.3 OpenCV
Tested with OpenCV 4.2.0 comes with ROS Noetic.[OpenCV installation](https://opencv.org/).
## 2.Compile

```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone vins_momo_M2DGRP
cd ../..
catkin_make
```
## 3.Run M2DGR example

```
source devel/setup.bash
roslaunch vins_estimator m2dgr.launch

source devel/setup.bash
roslaunch vins_estimator vins_rviz.launch

rosbag play door_02.bag
```
![vinsmomo_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/vins_momo_M2DGRP/image/Peek%202024-10-17%2023-00.gif)

## 4.Run M2DGR-plus example
```
source devel/setup.bash
roslaunch vins_estimator m2dgr_plus.launch 
    
source devel/setup.bash
roslaunch vins_estimator vins_rviz.launch

rosbag play tree3.bag --topic /camera/color/image_raw /camera/imu 
```
![vinsmomo_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/vins_momo_M2DGRP/image/plus.gif)

