# A-LOAM


## run
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
    source devel/setup.bash
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    # 在bag所在文件夹下打开终端，开始播放数据集：
    rosbag play xx.bag /rslidar_points:=/velodyne_point
```


