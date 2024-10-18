# LeGO-LOAM

## Dependency
```
sudo apt-get install libparmetis-dev
```
gtsam4.0.3
```
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
ceres1.14.0
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

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone LIO_Sam_M2DGRP
cd ../..
catkin_make
```
## Run M2DGR example：
```
//todo: add .yaml file
```

## Run M2DGR-plus example：
```
source devel/setup.bash

roslaunch lego_loam run.launch

rosbag play tree3.bag --clock --topics /rslidar_points /rslidar_points:=/velodyne_points  //todo:add imu
```
![lego_loam_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/Lego_loam_M2DGRP/image/plus.gif)

