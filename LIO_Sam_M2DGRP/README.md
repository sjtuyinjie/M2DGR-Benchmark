## Run the package

1. Run the launch file:
```
catkin_make
source devel/setup.bash
roslaunch lio_sam run.launch
```

2. Play existing bag files:
```
rosbag play tree3.bag --topic /rslidar_points /camera/imu
```
