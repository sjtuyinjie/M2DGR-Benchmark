

# M2DGR-Benchmark

**Contributors** of this repo: Junjie Zhang (张骏杰), Deteng Zhang (张德腾), Yan Sun (孙岩), and [Jie Yin (殷杰)](https://sjtuyinjie.github.io/)*

[![ C-Author](https://img.shields.io/badge/Author-Jie%20Yin-blue)](https://sjtuyinjie.github.io/)
[![Dataset](https://img.shields.io/badge/Dataset-M2DGR-yellow)](https://github.com/SJTU-ViSYS/M2DGR)
[![Dataset](https://img.shields.io/badge/DatasetExtension-M2DGR+%2B-green)](https://github.com/SJTU-ViSYS/M2DGR-plus)
[![License](https://img.shields.io/badge/License-GPLv3-cyan)]()

The goal of **M2DGR-Benchmark** is to **benchmark all cutting-edge SLAM systems on M2DGR/M2DGR+ datasets**! So we will keep updating  state-of-the-art SLAM systems to them.

- [**M2DGR**](https://github.com/SJTU-ViSYS/M2DGR): A Multi-modal and Multi-scenario SLAM Dataset for Ground Robots (RA-L & ICRA 2022).
- [**M2DGR-plus**](https://github.com/SJTU-ViSYS/M2DGR-plus): An extension of M2DGR (ICRA 2022 & ICRA 2024).

This project adapts leading LiDAR, Visual, and sensor-fusion SLAM systems to **both M2DGR and M2DGR+** dataset, facilitating research and development in SLAM technologies. Detailed installation methods of open-source projects on M2DGR/M2DGR+ are available in the respective project folders. 

Furthermore, extensive open-source systems are tested upon M2DGR, such as [**Ground-Fusion**](https://github.com/SJTU-ViSYS/Ground-Fusion), [LVI-SAM-Easyused](https://github.com/Cc19245/LVI-SAM-Easyused), [MM-LINS](https://github.com/lian-yue0515/MM-LINS/), [Log-LIO](https://github.com/tiev-tongji/LOG-LIO), [LIGO](https://github.com/Joanna-HE/LIGO.), [Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM), [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public), [GRIL-Cali](https://github.com/Taeyoung96/GRIL-Calib), [LINK3d](https://github.com/YungeCui/LinK3D), [i-Octree](https://github.com/zhujun3753/i-octree), [LIO-EKF](https://github.com/YibinWu/LIO-EKF), [Fast-LIO ROS2](https://github.com/Lee-JaeWon/FAST_LIO_ROS2), [HC-LIO](https://github.com/piluohong/hc_lio), [LIO-RF](https://github.com/YJZLuckyBoy/liorf), [PIN-SLAM](https://github.com/PRBonn/PIN_SLAM), [LOG-LIO2](https://github.com/tiev-tongji/LOG-LIO2), [Section-LIO](https://github.com/mengkai98/Section-LIO), [I2EKF-LO](https://github.com/YWL0720/I2EKF-LO), [Liloc](https://github.com/Yixin-F/LiLoc), [BMBL](https://github.com/YixFeng/Block-Map-Based-Localization), [Light-LOAM](https://github.com/BrenYi/Light-LOAM) and so on. Feel free to try these on M2DGR-benchmark!

![Example of r3live_on M2DGR door02](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/r3live_M2DGRP/image/Peek%202024-10-17%2022-30.gif)

## LiDAR-based Methods

- [**A-LOAM**](https://github.com/HKUST-Aerial-Robotics/A-LOAM)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/A_LOAM_M2DGRP) in the `A_LOAM_M2DGRP` folder.  
  *Description*: A-LOAM is an advanced implementation of the LOAM (Lidar Odometry and Mapping) algorithm, which simplifies code structure using Eigen and Ceres Solver. A-LOAM is clean, concise, and well-suited for SLAM beginners.

- [**LINS**](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LINS_M2DGRP) in the `LINS_M2DGRP` folder.  
  *Description*: LINS is a tightly-coupled lidar-inertial odometry and mapping system designed for robust real-time performance, especially in feature-less environments. It integrates IMU data with LiDAR scans to improve mapping and localization accuracy, particularly with Velodyne VLP-16.

- [**LIO-SAM**](https://github.com/TixiaoShan/LIO-SAM)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LIO_Sam_M2DGRP) in the `LIO_Sam_M2DGRP` folder.  
  *Description*: LIO-SAM is a real-time lidar-inertial odometry system that uses two factor graphs: one for optimizing lidar odometry and GPS data, and another for IMU data. This dual-graph system enables fast, accurate odometry and efficient map optimization.

- [**LeGO-LOAM**](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/Lego_loam_M2DGRP) in the `Lego_loam_M2DGRP` folder.  
  *Description*: LeGO-LOAM is a lightweight, ground-optimized lidar odometry and mapping system, designed for unmanned ground vehicles (UGVs). It provides real-time 6D pose estimation using a Velodyne VLP-16 LiDAR and optional IMU data.



## Vision-based Methods

- [**VINS-Mono**](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/vins_mono_M2DGRP) in the `vins_mono_M2DGRP` folder.  
  *Description*: VINS-Mono is a real-time monocular visual-inertial SLAM framework. It provides high-accuracy visual-inertial odometry using an optimization-based sliding window method, along with features like loop detection, failure recovery, and global pose graph optimization.


## LiDAR-Visual Fusion Methods

- [**FAST-LIVO**](https://github.com/hku-mars/FAST-LIVO)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/FAST_livo_M2DGRP) in the `FAST_livo_M2DGRP` folder.  
  *Description*: FAST-LIVO is a fast LiDAR-Inertial-Visual odometry system that registers raw point clouds and uses direct photometric alignment to minimize visual errors. It ensures robust odometry by tightly coupling the visual and lidar subsystems without relying on traditional feature extraction.

- [**LVI-SAM**](https://github.com/TixiaoShan/LVI-SAM)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/LVI_Sam_M2DGRP) in the `LVI_Sam_M2DGRP` folder.  
  *Description*: LVI-SAM combines the strengths of LIO-SAM and VINS-Mono to provide a robust, real-time SLAM system. It integrates lidar, visual, and inertial data for efficient odometry and mapping.

- [**R2LIVE**](https://github.com/hku-mars/r2live)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/r2live_M2DGRP) in the `r2live_M2DGRP` folder.  
  *Description*: R2LIVE is a tightly-coupled, real-time LiDAR-Inertial-Visual odometry and mapping system. It provides robust state estimation through precise sensor fusion and operates efficiently in challenging environments.

- [**R3LIVE**](https://github.com/hku-mars/r3live)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/r3live_M2DGRP) in the `r3live_M2DGRP` folder.  
  *Description*: R3LIVE builds upon R2LIVE, integrating both visual-inertial and lidar-inertial odometry for accurate state estimation. The system creates highly detailed 3D maps by combining lidar data for geometry and visual data for texture.

- [**SR-LIVO**](https://github.com/ZikangYuan/sr_livo)  
  [Documentation](https://github.com/sjtuyinjie/M2DGR-Benchmark/blob/main/sr_livo_M2DGRP) in the `sr_livo_M2DGRP` folder.  
  *Description*: SR-LIVO, based on R3LIVE, introduces sweep reconstruction to better align lidar data with image timestamps. This technique improves both pose accuracy and computational efficiency, resulting in precise colored point cloud maps.



Explore these methods to see how they perform on the M2DGR and M2DGR-plus datasets!



## License

If you use this work in an academic work, please cite:
~~~
@article{yin2021m2dgr,
  title={M2dgr: A multi-sensor and multi-scenario slam dataset for ground robots},
  author={Yin, Jie and Li, Ang and Li, Tao and Yu, Wenxian and Zou, Danping},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={2},
  pages={2266--2273},
  year={2021},
  publisher={IEEE}
}

@article{yin2024ground,
  title={Ground-Fusion: A Low-cost Ground SLAM System Robust to Corner Cases},
  author={Yin, Jie and Li, Ang and Xi, Wei and Yu, Wenxian and Zou, Danping},
  journal={arXiv preprint arXiv:2402.14308},
  year={2024}
}
~~~


## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=sjtuyinjie/M2DGR-Benchmark&type=Timeline)](https://star-history.com/#Ashutosh00710/github-readme-activity-graph&Timeline)

