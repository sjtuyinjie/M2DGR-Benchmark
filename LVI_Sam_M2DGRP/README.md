# [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)
This code can use six-axis or nine-axis imu. If the lidar information only contains position information, manually add timestamp and ring information to it to ensure the accurate operation of the code.
## 1.Dependency
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
### 1.4 Ceres 
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
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone LVI_Sam_M2DGRP
cd M2DGR-Benchmark/LVI_Sam_M2DGRP
mkdir build && cd build
cmake ..
make -j4 
```

## 3.Run M2DGR example
```
source devel/setup.bash

roslaunch lvi_sam my_run.launch

rosbag play door_02.bag
```
![lvi_sam_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LVI_Sam_M2DGRP/image/Peek%202024-10-13%2020-02.gif)
## Run 4.M2DGR-plus example
```
source devel/setup.bash

roslaunch lvi_sam plus_run.launch

rosbag play tree3.bag --topic /rslidar_points /camera/color/image_raw /camera/imu
```
![lvi_sam_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LVI_Sam_M2DGRP/image/plus.gif)

