# [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
This file runs LeGO-LOAM on M2DGR and M2DGR-plus.
## 1.Dependency
### 1.1 Ubuntu and ROS
Tested with Ubuntu 20.04 and ROS Noetic.[ROS Installation](https://wiki.ros.org/ROS/Installation).
### 1.2 [GTSAM](https://gtsam.org/get_started/)(Georgia Tech Smoothing and Mapping library)
Tested with gtsam 4.0.3.
```
sudo apt-get install libparmetis-dev
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
### 1.3 Ceres 
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

## 2.Compile
You can use the following commands to download and compile the package.
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone LIO_Sam_M2DGRP
cd ../..
catkin_make
```
## 3.Run M2DGR example：
First, comment out lines `62-68` in the [utility.h](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/Lego_loam_M2DGRP/lego_loam_m2dgrp/include/utility.h) file and uncomment lines `71-76` to change the lidar parameters. And change line 20 to ```extern const bool useCloudRing = true;```. Recompile it.
```
catkin_make

source devel/setup.bash
roslaunch lego_loam m2dgr_run.launch

//lidar
rosbag play door_02.bag --clock --topics /velodyne_points

//lidar+imu
rosbag play door_02.bag --clock --topics /velodyne_points /handsfree/imu:=/imu/data
```
![lego_loam_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/Lego_loam_M2DGRP/image/Peek%202024-10-22%2023-13.gif)
## 4.Run M2DGR-plus example：
```
source devel/setup.bash

roslaunch lego_loam m2dgr_plus_run.launch

//lidar
rosbag play tree3.bag --clock --topics /rslidar_points /rslidar_points:=/velodyne_points

//lidar+imu
rosbag play tree3.bag --clock --topics /rslidar_points /rslidar_points:=/velodyne_points /imu:=/imu/data
```
![lego_loam_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/Lego_loam_M2DGRP/image/plus.gif)

