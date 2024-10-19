# [R2live](https://github.com/hku-mars/r2live) 
This file uses external coordinate conversion and lidar format conversion to convert the M2DGR-plus lidar and IMU to a coordinate system. Adds timestamps and ring information to the lidar data to transform `/rslidar_points` into `/velodyne_points` format.
## 1.Dependency
### 1.1 Ubuntu and ROS
Tested with Ubuntu 20.04 and ROS Noetic.[ROS Installation](https://wiki.ros.org/ROS/Installation).
### 1.2 Livox SDK
Because need to install livox_ros_driver, install the livox SDK first.
```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install
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
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone r2live_M2DGRP
cd ../..
catkin_make
```
## 3.Run M2DGR example
```
source devel/setup.bash
roslaunch r2live m2dgr.launch
rosbag play door_02.bag 
```
![r2live_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/r2live_M2DGRP/image/Peek%202024-10-17%2022-08.gif)
## 4.Run M2DGR-plus example
```
source devel/setup.bash
roslaunch r2live m2dgr_plus.launch

//imu coordinate conversion
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

//lidar format conversion
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play anomaly.bag --topic /rslidar_points /camera/color/image_raw /camera/imu

```
![r2live_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/r2live_M2DGRP/image/plus.gif)

