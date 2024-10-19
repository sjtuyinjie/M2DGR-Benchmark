# [R3live](https://github.com/hku-mars/r3live)
This file uses external coordinate conversion and lidar format conversion to convert the M2DGR-plus lidar and IMU to a coordinate system. Adds timestamps and ring information to the lidar data to transform `/rslidar_points` into `/velodyne_points` format.
## 1.Dependency
### 1.1 Ubuntu and ROS
Tested with Ubuntu 20.04 and ROS Noetic.[ROS Installation](https://wiki.ros.org/ROS/Installation).
### 1.2 CGAL && tf2
```
sudo apt-get install libcgal-dev pcl-tools
sudo apt-get install ros-noetic-tf2-sensor-msgs
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
### 1.4 OpenCV
Tested with OpenCV 4.2.0 comes with ROS Noetic.[OpenCV installation](https://opencv.org/).

## 2.Compile

```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone r3live_M2DGRP
cd ../..
catkin_make
```

## 3.Run M2DGR example

```
source devel/setup.bash

roslaunch r3live r3live_m2dgr.launch

rosbag play door_02.bag
```
![r3live_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/r3live_M2DGRP/image/Peek%202024-10-17%2022-30.gif)

## 4.Run M2DGR-plus example
```
source devel/setup.bash
roslaunch r3live r3live_m2dgr_plus_v.launch

//imu coordinate conversion
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

//lidar format conversion
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play tree3.bag --topic /rslidar_points /camera/color/image_raw /camera/imu

```
![r3live_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/r3live_M2DGRP/image/plus.gif)

