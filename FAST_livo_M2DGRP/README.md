# [FAST-LIVO](https://github.com/hku-mars/FAST-LIVO?tab=readme-ov-file)
##  Dependency
### Ubuntu and ROS
Ubuntu 20.04 and ROS Noetic.[ROS Installation](https://wiki.ros.org/ROS/Installation).
### Eigen
Eigen 3.3.7.[Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).
### PCL
PCL 1.10 comes with ROS Noetic.[PCL installation](https://pointclouds.org/).
### OpenCV
OpenCV 4.2.0 comes with ROS Noetic.[PCL installation](https://pointclouds.org/).
### Sophus
Sophus Installation for the non-templated/double-only version.
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```
### livox SDK
Because need to install livox_ros_driver, install the livox SDK first.
```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install
```
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

//imu coordinate conversion
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

//lidar format conversion
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play anomaly.bag --topic /rslidar_points /camera/color/image_raw /camera/imu
```
![fast_livo_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/FAST_livo_M2DGRP/image/plus.gif)
