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
#include "utilities.hpp"
#include "loop_closure.h"

namespace fs = std::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLioSamQn
{
  private:
    ///// basic params
    std::string m_map_frame;
    std::string m_package_path;
    std::string m_seq_name;
    ///// shared data - odom and pcd
    std::mutex m_realtime_pose_mutex, m_keyframes_mutex;
    std::mutex m_graph_mutex, m_vis_mutex;
    Eigen::Matrix4d m_last_corrected_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d m_odom_delta = Eigen::Matrix4d::Identity();
    PosePcd m_current_frame, m_not_processed_keyframe;
    std::vector<PosePcd> m_keyframes;
    int m_current_keyframe_idx = 0;
    bool m_init = false;
    ///// graph and values
    shared_ptr<gtsam::ISAM2> m_isam_handler = nullptr;
    gtsam::NonlinearFactorGraph m_gtsam_graph;
    gtsam::Values m_init_esti;
    gtsam::Values m_corrected_esti;
    double m_keyframe_thr;
    ///// loop
    
    double m_voxel_res;
    int m_sub_key_num;
    std::vector<std::pair<size_t, size_t>> m_loop_idx_pairs; //for vis
    bool m_loop_added_flag = false; //for opt
    bool m_loop_added_flag_vis = false; //for vis
    ///// visualize
    tf::TransformBroadcaster m_broadcaster;
    pcl::PointCloud<pcl::PointXYZ> m_odoms, m_corrected_odoms;
    nav_msgs::Path m_odom_path, m_corrected_path;
    bool m_global_map_vis_switch = true;
    ///// results
    bool m_save_map_bag = false, m_save_map_pcd = false, m_save_in_kitti_format = false;
    ///// ros
    ros::NodeHandle m_nh;
    ros::Publisher m_corrected_odom_pub, m_corrected_path_pub, m_odom_pub, m_path_pub;
    ros::Publisher m_corrected_current_pcd_pub, m_corrected_pcd_map_pub, m_loop_detection_pub;
    ros::Publisher m_realtime_pose_pub;
    ros::Publisher m_debug_src_pub, m_debug_dst_pub, m_debug_coarse_aligned_pub, m_debug_fine_aligned_pub;
    ros::Timer m_loop_timer, m_vis_timer;
    // odom, pcd sync, and save flag subscribers
    shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> m_sub_odom_pcd_sync = nullptr;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> m_sub_odom = nullptr;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_sub_pcd = nullptr;
    ros::Subscriber m_sub_save_flag;

    LoopClosureConfig lc_config_;
    std::unique_ptr<LoopClosure> loop_closure_;
    ///// functions
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //note for Eigen alignment, this might not be necessary from C++17

    FastLioSamQn(const ros::NodeHandle& n_private);
    ~FastLioSamQn();
  private:
    //methods
    void updateVisVars(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    //cb
    void odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    void saveFlagCallback(const std_msgs::String::ConstPtr &msg);
    void loopTimerFunc(const ros::TimerEvent& event);
    void visTimerFunc(const ros::TimerEvent& event);
};


#endif
