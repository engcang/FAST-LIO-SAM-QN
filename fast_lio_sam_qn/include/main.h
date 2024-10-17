#ifndef FAST_LIO_SAM_QN_MAIN_H
#define FAST_LIO_SAM_QN_MAIN_H

///// coded headers
#include "utilities.h"
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
#include <filesystem>
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
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/common/transforms.h> //transformPointCloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/filters/voxel_grid.h> //voxelgrid
#include <pcl/io/pcd_io.h> // save map
///// Nano-GICP
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>
///// Quatro
#include <quatro/quatro_module.h>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
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

using namespace std::chrono;
using PointType = pcl::PointXYZI;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;


////////////////////////////////////////////////////////////////////////////////////////////////////
struct PosePcd
{
  pcl::PointCloud<PointType> pcd;
  Eigen::Matrix4d pose_eig = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_corrected_eig = Eigen::Matrix4d::Identity();
  double timestamp;
  int idx;
  bool processed = false;
  PosePcd(){};
  PosePcd(const nav_msgs::Odometry &odom_in, const sensor_msgs::PointCloud2 &pcd_in, const int &idx_in);
};

struct RegistrationOutput
{
  Eigen::Matrix4d pose_between_eig = Eigen::Matrix4d::Identity();
  bool is_converged                = false;
  double score                     = std::numeric_limits<float>::max();
};
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
    pcl::VoxelGrid<PointType> m_voxelgrid, m_voxelgrid_vis;
    nano_gicp::NanoGICP<PointType, PointType> m_nano_gicp;
    shared_ptr<quatro<PointType>> m_quatro_handler = nullptr;
    bool m_enable_submap_matching = false;
    bool m_enable_quatro = false;
    double m_icp_score_thr, m_loop_det_radi, m_loop_det_tdiff_thr;
    int m_sub_key_num;
    std::vector<pair<int, int>> m_loop_idx_pairs; //for vis
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

    ///// functions
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FastLioSamQn(const ros::NodeHandle& n_private);
    ~FastLioSamQn();
  private:
    //methods
    void updateVisVars(const PosePcd &pose_pcd_in);
    void voxelizePcd(pcl::VoxelGrid<PointType> &voxelgrid, pcl::PointCloud<PointType> &pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    int getClosestKeyframeIdx(const PosePcd &front_keyframe, const std::vector<PosePcd> &keyframes);
    std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>> setSrcAndDstCloud(std::vector<PosePcd> keyframes, const int src_idx, const int dst_idx, const int submap_range, const bool enable_quatro, const bool enable_submap_matching);
    RegistrationOutput icpAlignment(const pcl::PointCloud<PointType> &src_raw_, const pcl::PointCloud<PointType> &dst_raw_);
    RegistrationOutput coarseToFineAlignment(const pcl::PointCloud<PointType> &src_raw_, const pcl::PointCloud<PointType> &dst_raw_);
    visualization_msgs::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    //cb
    void odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    void SaveFlagCallback(const std_msgs::String::ConstPtr &msg);
    void loopTimerFunc(const ros::TimerEvent& event);
    void visTimerFunc(const ros::TimerEvent& event);
};



#endif