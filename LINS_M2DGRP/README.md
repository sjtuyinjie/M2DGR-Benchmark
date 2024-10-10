# m2dgr_plus


```
source devel/setup.bash
roslaunch lins run_m2dgr_plus.launch

# 开启imu坐标转换
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

# 开启lidar格式转换
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play tree3.bag --clock --topic /rslidar_points /camera/color/image_raw /camera/imu

```
