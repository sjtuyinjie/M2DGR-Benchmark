# compile
```
//安装livox_SDK
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install

sudo apt-get install libcgal-dev pcl-tools//安装CGAL
sudo apt-get install ros-noetic-tf2-sensor-msgs // 安装tf2
catkin_make
```
# run

##  m2dgr
```
source devel/setup.bash
roslaunch r3live r3live_m2dgr.launch
rosbag play xx.bag
```

##  m2dgr-plus
该方法将Robosense转换为velodyne雷达话题格式发布，包含ring、time
```
source devel/setup.bash
roslaunch r3live r3live_m2dgr_plus_v.launch

# 开启imu坐标转换
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

# 开启lidar格式转换
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play tree3.bag --topic /rslidar_points /camera/color/image_raw /camera/imu

```
