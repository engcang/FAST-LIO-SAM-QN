# Note
+ 2023-07-12: The code is not implemented yet. The current code is just copy-pasted version of [former version](https://github.com/engcang/FAST-LIO-SAM). I am planning to finish this within this weekend.

<br>

# FAST-LIO-SAM-QN
+ This repository is a SLAM implementation combining [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) with pose graph optimization and loop closing based on [Quatro](https://github.com/url-kaist/Quatro) and [Nano-GICP](https://github.com/vectr-ucla/direct_lidar_odometry)
    + [Quatro](https://github.com/url-kaist/Quatro) - fast, accurate and robust global registration which provides great initial guess of transform
    + [Nano-GICP](https://github.com/vectr-ucla/direct_lidar_odometry) - fast and accurate ICP combining [FastGICP](https://github.com/SMRT-AIST/fast_gicp) + [NanoFLANN](https://github.com/jlblancoc/nanoflann)
+ Note: similar repositories already exist
    + [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC): FAST-LIO2 + SC-A-LOAM based SLAM
    + [FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM): FAST-LIO2 + ScanContext based SLAM
    + [FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM): FAST-LIO2 + LIO-SAM (not modularized)
    + [FAST_LIO_SAM](https://github.com/engcang/FAST-LIO-SAM): FAST-LIO2 + LIO-SAM (modularized)
+ Note2: main code (PGO) is modularized and hence can be combined with any other LIO / LO
    + This repo is to learn GTSAM myself!
    + and as GTSAM tutorial for beginners - [GTSAM 튜토리얼 한글 포스팅](https://engcang.github.io/2023/07/15/gtsam_tutorial.html)

<br>

## Requirements
+ ROS (it comes with `Eigen` and `PCL`)
+ [GTSAM](https://github.com/borglab/gtsam)
    ```shell
    wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
    unzip gtsam.zip
    cd gtsam-4.1.1/
    mkdir build && cd build
    cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
    sudo make install -j16
    ```

## How to build and use
+ Get the code, build `tbb` first, and then build the main code
    + `tbb` is only used for faster `pcl::transformPointCloud`, you can just remove it by replacing `tf_pcd` with `pcl::transformPointCloud`
    ```shell
    cd ~/your_workspace/src
    git clone https://github.com/engcang/FAST-LIO-SAM-QN --recursive

    cd FAST-LIO-SAM-QN/third_party/tbb-aarch64
    ./scripts/bootstrap-aarch64-linux.sh
    cd build-aarch64
    make -j16 && make install

    cd ~/your_workspace
    catkin build -DCMAKE_BUILD_TYPE=Release
    . devel/setup.bash
    ```
+ Then run (change config files in third_party/`FAST_LIO`)
    ```shell
    roslaunch fast_lio_sam_qn run.launch lidar:=ouster
    roslaunch fast_lio_sam_qn run.launch lidar:=velodyne
    roslaunch fast_lio_sam_qn run.launch lidar:=livox
    ```

<br>

### Structure
+ odom_pcd_cb
    + pub realtime pose in corrected frame
    + keyframe detection -> if keyframe, add to pose graph + save to keyframe queue
    + pose graph optimization with iSAM2
+ loop_timer_func
    + process a saved keyframe
        + detect loop -> if loop, add to pose graph
+ vis_timer_func
    + visualize all **(Note: global map is only visualized once uncheck/check the mapped_pcd in rviz to save comp.)**