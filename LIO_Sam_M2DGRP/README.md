# LIO_Sam


## Dependency
livox SDK安装
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
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone LIO_Sam_M2DGRP
cd ../..
catkin_make
```

## Run M2DGR example

```
source devel/setup.bash

roslaunch lio_sam m2dgr_run.launch

rosbag play door_02.bag
```
![lio_sam_door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LIO_Sam_M2DGRP/image/Peek%202024-10-13%2013-57.gif)
## Run M2DGR-plus example

```
source devel/setup.bash

roslaunch lio_sam m2dgrplus_run.launch

rosbag play tree3.bag --topic /rslidar_points /camera/imu
```
![lio_sam_tree3](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LIO_Sam_M2DGRP/image/plus.gif)
