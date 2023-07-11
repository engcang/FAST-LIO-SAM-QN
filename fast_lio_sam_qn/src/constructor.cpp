#include "main.h"


pose_pcd::pose_pcd(const nav_msgs::Odometry &odom_in, const sensor_msgs::PointCloud2 &pcd_in, const int &idx_in)
{
  tf::Quaternion q_(odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y, odom_in.pose.pose.orientation.z, odom_in.pose.pose.orientation.w);
  tf::Matrix3x3 m_(q_);
  Eigen::Matrix3d tmp_rot_mat_;
  tf::matrixTFToEigen(m_, tmp_rot_mat_);
  pose_eig.block<3, 3>(0, 0) = tmp_rot_mat_;
  pose_eig(0, 3) = odom_in.pose.pose.position.x;
  pose_eig(1, 3) = odom_in.pose.pose.position.y;
  pose_eig(2, 3) = odom_in.pose.pose.position.z;
  pose_corrected_eig = pose_eig;
  pcl::PointCloud<pcl::PointXYZI> tmp_pcd_;
  pcl::fromROSMsg(pcd_in, tmp_pcd_);
  pcd = tf_pcd(tmp_pcd_, pose_eig.inverse()); //FAST-LIO publish data in world frame, so save it in LiDAR frame
  timestamp = odom_in.header.stamp.toSec();
  idx = idx_in;
}

FAST_LIO_SAM_QN_CLASS::FAST_LIO_SAM_QN_CLASS(const ros::NodeHandle& n_private) : m_nh(n_private)
{
  ////// ROS params
  // temp vars
  double loop_update_hz_, vis_hz_;
  // get params
  m_nh.param<string>("/map_frame", m_map_frame, "map");
  m_nh.param<double>("/keyframe_threshold", m_keyframe_thr, 1.0);
  m_nh.param<double>("/loop_detection_radius", m_loop_det_radi, 15.0);
  m_nh.param<double>("/loop_detection_timediff_threshold", m_loop_det_tdiff_thr, 10.0);
  m_nh.param<double>("/icp_score_threshold", m_icp_score_thr, 10.0);
  m_nh.param<int>("/subkeyframes_number", m_sub_key_num, 5);
  m_nh.param<double>("/loop_update_hz", loop_update_hz_, 1.0);
  m_nh.param<double>("/vis_hz", vis_hz_, 0.5);

  ////// GTSAM init
  gtsam::ISAM2Params isam_params_;
  isam_params_.relinearizeThreshold = 0.01;
  isam_params_.relinearizeSkip = 1;
  m_isam_handler = std::make_shared<gtsam::ISAM2>(isam_params_);
  ////// loop init
  m_voxelgrid.setLeafSize(0.3, 0.3, 0.3);
  m_voxelgrid_vis.setLeafSize(0.2, 0.2, 0.2);
  m_icp.setMaxCorrespondenceDistance(m_loop_det_radi*2.0);
  m_icp.setTransformationEpsilon(1e-2);
  m_icp.setEuclideanFitnessEpsilon(1e-2);
  m_icp.setMaximumIterations(100);
  m_icp.setRANSACIterations(0);

  ////// ROS things
  m_odom_path.header.frame_id = m_map_frame;
  m_corrected_path.header.frame_id = m_map_frame;
  // publishers
  m_odom_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/ori_odom", 10, true);
  m_path_pub = m_nh.advertise<nav_msgs::Path>("/ori_path", 10, true);
  m_corrected_odom_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/corrected_odom", 10, true);
  m_corrected_path_pub = m_nh.advertise<nav_msgs::Path>("/corrected_path", 10, true);
  m_corrected_pcd_map_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/corrected_map", 10, true);
  m_corrected_current_pcd_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/corrected_current_pcd", 10, true);
  m_loop_detection_pub = m_nh.advertise<visualization_msgs::Marker>("/loop_detection", 10, true);
  m_realtime_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/pose_stamped", 10);
  m_debug_src_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/src", 10, true);
  m_debug_dst_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/dst", 10, true);
  m_debug_aligned_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/aligned", 10, true);
  // subscribers
  m_sub_odom = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(m_nh, "/Odometry", 10);
  m_sub_pcd = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(m_nh, "/cloud_registered", 10);
  m_sub_odom_pcd_sync = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *m_sub_odom, *m_sub_pcd);
  m_sub_odom_pcd_sync->registerCallback(boost::bind(&FAST_LIO_SAM_QN_CLASS::odom_pcd_cb, this, _1, _2));
  // Timers at the end
  m_loop_timer = m_nh.createTimer(ros::Duration(1/loop_update_hz_), &FAST_LIO_SAM_QN_CLASS::loop_timer_func, this);
  m_vis_timer = m_nh.createTimer(ros::Duration(1/vis_hz_), &FAST_LIO_SAM_QN_CLASS::vis_timer_func, this);
  
  ROS_WARN("Main class, starting node...");
}