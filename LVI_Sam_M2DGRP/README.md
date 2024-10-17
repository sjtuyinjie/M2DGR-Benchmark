# LVI-SAM

## Dependency
ubuntu20.04
noetic
ceres1.14.0
gtsam4.0.3
opencv4

## Compile

```
git clone https://github.com/sjtuyinjie/M2DGR-Benchmark.git && cd M2DGR-Benchmark && git sparse-checkout set --no-cone LVI_Sam_M2DGRP
cd M2DGR-Benchmark/LVI_Sam_M2DGRP
mkdir build && cd build
cmake ..
make -j4 
```

## Run M2DGR example
```
source devel/setup.bash

roslaunch lvi_sam my_run.launch

rosbag play door_02.bag
```

## Run M2DGR-plus example
```
source devel/setup.bash

roslaunch lvi_sam plus_run.launch

rosbag play tree3.bag --topic /rslidar_points /camera/color/image_raw /camera/imu
```


## M2DGR-P....
lvi-sam 需要点云格式为PointXYZIRT，M2DGR-P云格式为PointXYZI，缺少ring timestamp
###dense flag:
```
//M2DGR-P最后is dense 参数为0。在imageprojection.cpp：194 "check dense flag"会拒绝点云输入
//采取办法：注释掉check dense flag部分
```

###ring:
```
//注释掉imageprojection.cpp：201 check ring channel部分

/*imageprojection.cpp：525、526增加手动计算ring，注释掉int rowIdn = laserCloudIn->points[i].ring;*/
verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
 int rowIdn = (verticalAngle + ang_bottom) / ang_res_y;

//在相应projectPointCloud()函数内增加定义
 float verticalAngle;
 const float ang_res_y = 2.5;
 const float ang_bottom = 30.0+0.1;  
 
```
参考：lego-loam
###timestamp
```
//注释掉imageprojection.cpp：221 check point time部分
238～272增加手动添加时间戳部分
```
参考：lego-loam
#更改后仍可运行原lvi-sam数据包，说明更改后新时间戳与ring应该不存在大问题。

