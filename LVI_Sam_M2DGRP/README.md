# LVI-SAM/M2DGR/M2DGRM-P

ubuntu20.04
noetic
ceres1.14.0
gtsam4.0.3
opencv4

## code
lvi-sam目录下的CMakelists.txt 更改c++11为c++14

所有替换
include <opencv/cv.h> -> include <opencv2/opencv.hpp>;
CV_RGB2GRAY->cv::COLOR_RGB2GRAY;
CV_FONT_HERSHEY_SIMPLEX->cv::FONT_HERSHEY_SIMPLEX;

LVI-SAM/src/visual_odometry/visual_loop/ThirdParty/DVision/BRIEF.cpp  添加头文件#include <opencv2/imgproc.hpp>
[参考](https://www.guyuehome.com/35709)

## Compile
若catkin_make崩溃，使用
```
make -j4 
```

## run
```
//M2DGR: 
roslaunch lvi_sam my_run.launch 
rosbag play xx.bag
//M2DGR-P: 
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

问题...
现在存在参考frame不一致问题，bag文件topic播放的frame与代码参考不一致。

bag文件中存在tf tree！与原算法冲突。两个base_link。
解决： rosbag filter origin.bag dest.bag "topic != '/tf'"
生成no_tf,bag
可以不进行过滤，但frame存在混乱问题，一定影响精度？
##标定
解决，去除平移量
rsimu2lidar=[0 0 1
      -1 0 0
       0 -1 0]

rsimu2camera=[1 0 0
      0 1 0
      0 0 1]

lidar2camera=[0 -1 0
      0 0 -1
      1 0 0]


