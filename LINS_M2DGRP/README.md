# [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM)
This file uses external coordinate conversion and lidar format conversion to convert the M2DGR-plus lidar and IMU to a coordinate system. Adds timestamps and ring information to the lidar data to transform `/rslidar_points` into `/velodyne_points` format.
##  1.Dependency
### 1.1 Ubuntu and ROS
Tested with Ubuntu 20.04 and ROS Noetic.[ROS Installation](https://wiki.ros.org/ROS/Installation).
### 1.2 [GTSAM](https://gtsam.org/get_started/)(Georgia Tech Smoothing and Mapping library)
Tested with gtsam 4.0.3.
```
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
### 1.3 OpenCV
Tested with OpenCV 4.2.0 comes with ROS Noetic.[OpenCV installation](https://opencv.org/).

## 2. Compile
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone LINS_M2DGRP
cd ../..
catkin_make
```

## 3. Run M2DGR example
```
source devel/setup.bash

roslaunch lins run_m2dgr.launch

rosbag play door_02.bag --clock
```
![LINS_door](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LINS_M2DGRP/image/Peek%202024-10-13%2011-49.gif)

## 4. Run M2DGR-plus example
```
source devel/setup.bash
roslaunch lins run_m2dgr_plus.launch

//imu coordinate conversion
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

//lidar format conversion
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play tree3.bag --clock --topic /rslidar_points /camera/color/image_raw /camera/imu
```
![LINS_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LINS_M2DGRP/image/plus.gif)

