#ifndef FAST_LIO_SAM_QN_MAIN_H
#define FAST_LIO_SAM_QN_MAIN_H

///// common headers
#include <time.h>
#include <math.h>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <deque>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
#include <tuple>
#include <filesystem>
#include <fstream>
#include <iostream>
///// ROS
#include <ros/ros.h>
#include <ros/package.h> // get package_path
#include <rosbag/bag.h> // save map
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler
#include <tf/transform_datatypes.h> // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h> // tf <-> eigen
#include <tf/transform_broadcaster.h> // broadcaster
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
///// coded headers
#include "loop_closure.h"
#include "pose_pcd.hpp"
#include "utilities.hpp"

namespace fs = std::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLioSamQn
{
  private:
    ///// basic params
    std::string map_frame_;
    std::string package_path_;
    std::string seq_name_;
    ///// shared data - odom and pcd
    std::mutex realtime_pose_mutex_, keyframes_mutex_;
    std::mutex graph_mutex_, vis_mutex_;
    Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
    PosePcd current_frame_;
    std::vector<PosePcd> keyframes_;
    int current_keyframe_idx_ = 0;
    bool is_initialized_ = false;
    ///// graph and values
    std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;
    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values init_esti_;
    gtsam::Values corrected_esti_;
    double keyframe_thr_;
    double voxel_res_;
    int sub_key_num_;
    std::vector<std::pair<size_t, size_t>> loop_idx_pairs_; //for vis
    bool loop_added_flag_ = false; //for opt
    bool loop_added_flag_vis_ = false; //for vis
    ///// visualize
    tf::TransformBroadcaster broadcaster_;
    pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
    nav_msgs::Path odom_path_, corrected_path_;
    bool global_map_vis_switch_ = true;
    ///// results
    bool save_map_bag_ = false, save_map_pcd_ = false, save_in_kitti_format_ = false;
    ///// ros
    ros::NodeHandle nh_;
    ros::Publisher corrected_odom_pub_, corrected_path_pub_, odom_pub_, path_pub_;
    ros::Publisher corrected_current_pcd_pub_, corrected_pcd_map_pub_, loop_detection_pub_;
    ros::Publisher realtime_pose_pub_;
    ros::Publisher debug_src_pub_, debug_dst_pub_, debug_coarse_aligned_pub_, debug_fine_aligned_pub_;
    ros::Timer loop_timer_, vis_timer_;
    // odom, pcd sync, and save flag subscribers
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_pcd_ = nullptr;
    ros::Subscriber sub_save_flag_;
    ///// Loop closure
    std::shared_ptr<LoopClosure> loop_closure_;
  public:
    FastLioSamQn(const ros::NodeHandle& n_private);
    ~FastLioSamQn();
  private:
    //methods
    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    //cb
    void odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    void saveFlagCallback(const std_msgs::String::ConstPtr &msg);
    void loopTimerFunc(const ros::TimerEvent& event);
    void visTimerFunc(const ros::TimerEvent& event);
};


#endif
