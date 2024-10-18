#include "fast_lio_sam_qn.h"


PosePcd::PosePcd(const nav_msgs::Odometry &odom_in, const sensor_msgs::PointCloud2 &pcd_in, const int &idx_in)
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
  pcl::PointCloud<PointType> tmp_pcd_;
  pcl::fromROSMsg(pcd_in, tmp_pcd_);
  pcd = transformPcd(tmp_pcd_, pose_eig.inverse()); //FAST-LIO publish data in world frame, so save it in LiDAR frame
  timestamp = odom_in.header.stamp.toSec();
  idx = idx_in;
}

FastLioSamQn::FastLioSamQn(const ros::NodeHandle& n_private) : m_nh(n_private)
{
  ////// ROS params
  // temp vars, only used in constructor
  double loop_update_hz_, vis_hz_;

  auto & gc = lc_config_.gicp_config_;
  auto & qc = lc_config_.quatro_config_;
  // get params
  /* basic */
  m_nh.param<std::string>("/basic/map_frame", m_map_frame, "map");
  m_nh.param<double>("/basic/loop_update_hz", loop_update_hz_, 1.0);
  m_nh.param<double>("/basic/vis_hz", vis_hz_, 0.5);
  m_nh.param<double>("/save_voxel_resolution", m_voxel_res, 0.3);
  m_nh.param<double>("/quatro_nano_gicp_voxel_resolution", lc_config_.voxel_res_, 0.3);
  /* keyframe */
  m_nh.param<double>("/keyframe/keyframe_threshold", m_keyframe_thr, 1.0);
  m_nh.param<int>("/keyframe/subkeyframes_number", m_sub_key_num, 5);
  m_nh.param<bool>("/keyframe/enable_submap_matching", lc_config_.enable_submap_matching_, false);
  
  /* loop */
  m_nh.param<double>("/loop/loop_detection_radius", lc_config_.loop_detection_radius_, 15.0);
  m_nh.param<double>("/loop/loop_detection_timediff_threshold", lc_config_.loop_detection_timediff_threshold_, 10.0);

  /* nano (GICP config) */
  m_nh.param<int>("/nano_gicp/thread_number", gc.nano_thread_number_, 0);
  m_nh.param<double>("/nano_gicp/icp_score_threshold", gc.icp_score_thr_, 10.0);
  m_nh.param<int>("/nano_gicp/correspondences_number", gc.nano_correspondences_number_, 15);
  m_nh.param<int>("/nano_gicp/max_iter", gc.nano_max_iter_, 32);
  m_nh.param<double>("/nano_gicp/transformation_epsilon", gc.transformation_epsilon_, 0.01);
  m_nh.param<double>("/nano_gicp/euclidean_fitness_epsilon", gc.euclidean_fitness_epsilon_, 0.01);
  m_nh.param<int>("/nano_gicp/ransac/max_iter", gc.nano_ransac_max_iter_, 5);
  m_nh.param<double>("/nano_gicp/ransac/outlier_rejection_threshold", gc.ransac_outlier_rejection_threshold_, 1.0);

  /* quatro (Quatro config) */
  m_nh.param<bool>("/quatro/enable", lc_config_.enable_quatro_, false);
  m_nh.param<bool>("/quatro/optimize_matching", qc.use_optimized_matching_, true);
  m_nh.param<double>("/quatro/distance_threshold", qc.quatro_distance_threshold_, 30.0);
  m_nh.param<int>("/quatro/max_num_correspondences", qc.quatro_max_num_corres_, 200);
  m_nh.param<double>("/quatro/fpfh_normal_radius", qc.fpfh_normal_radius_, 0.3);
  m_nh.param<double>("/quatro/fpfh_radius", qc.fpfh_radius_, 0.5);
  m_nh.param<bool>("/quatro/estimating_scale", qc.estimat_scale_, false);
  m_nh.param<double>("/quatro/noise_bound", qc.noise_bound_, 0.3);
  m_nh.param<double>("/quatro/rotation/gnc_factor", qc.rot_gnc_factor_, 1.4);
  m_nh.param<double>("/quatro/rotation/rot_cost_diff_threshold", qc.rot_cost_diff_thr_, 0.0001);
  m_nh.param<int>("/quatro/rotation/num_max_iter", qc.quatro_max_iter_, 50);
  
  /* results */
  m_nh.param<bool>("/result/save_map_bag", m_save_map_bag, false);
  m_nh.param<bool>("/result/save_map_pcd", m_save_map_pcd, false);
  m_nh.param<bool>("/result/save_in_kitti_format", m_save_in_kitti_format, false);
  m_nh.param<std::string>("/result/seq_name", m_seq_name, "");

  loop_closure_.reset(new LoopClosure(lc_config_));
  ////// GTSAM init
  gtsam::ISAM2Params isam_params_;
  isam_params_.relinearizeThreshold = 0.01;
  isam_params_.relinearizeSkip = 1;
  m_isam_handler = std::make_shared<gtsam::ISAM2>(isam_params_);

  ////// ROS things
  m_odom_path.header.frame_id = m_map_frame;
  m_corrected_path.header.frame_id = m_map_frame;
  m_package_path = ros::package::getPath("fast_lio_sam_qn");
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
  m_debug_coarse_aligned_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/coarse_aligned_quatro", 10, true);
  m_debug_fine_aligned_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/fine_aligned_nano_gicp", 10, true);
  // subscribers
  m_sub_odom = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(m_nh, "/Odometry", 10);
  m_sub_pcd = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(m_nh, "/cloud_registered", 10);
  m_sub_odom_pcd_sync = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *m_sub_odom, *m_sub_pcd);
  m_sub_odom_pcd_sync->registerCallback(boost::bind(&FastLioSamQn::odomPcdCallback, this, _1, _2));
  m_sub_save_flag = m_nh.subscribe("/save_dir", 1, &FastLioSamQn::SaveFlagCallback, this);
  // Timers at the end
  m_loop_timer = m_nh.createTimer(ros::Duration(1/loop_update_hz_), &FastLioSamQn::loopTimerFunc, this);
  m_vis_timer = m_nh.createTimer(ros::Duration(1/vis_hz_), &FastLioSamQn::visTimerFunc, this);

  ROS_WARN("Main class, starting node...");
}

FastLioSamQn::~FastLioSamQn()
{
  // save map
  if (m_save_map_bag)
  {
    rosbag::Bag bag_;
    bag_.open(m_package_path+"/result.bag", rosbag::bagmode::Write);
    {
      std::lock_guard<std::mutex> lock(m_keyframes_mutex);
      for (int i = 0; i < m_keyframes.size(); ++i)
      {
        ros::Time time_;
        time_.fromSec(m_keyframes[i].timestamp);
        bag_.write("/keyframe_pcd", time_, pclToPclRos(m_keyframes[i].pcd, m_map_frame));
        bag_.write("/keyframe_pose", time_, poseEigToPoseStamped(m_keyframes[i].pose_corrected_eig));
      }
    }
    bag_.close();
    cout << "\033[36;1mResult saved in .bag format!!!\033[0m" << endl;
  }
  if (m_save_map_pcd)
  {
    pcl::PointCloud<PointType> corrected_map_;
    {
      std::lock_guard<std::mutex> lock(m_keyframes_mutex);
      for (int i = 0; i < m_keyframes.size(); ++i)
      {
        corrected_map_ += transformPcd(m_keyframes[i].pcd, m_keyframes[i].pose_corrected_eig);
      }
    }
    pcl::io::savePCDFileASCII<PointType> (m_package_path+"/result.pcd", corrected_map_);
    cout << "\033[32;1mResult saved in .pcd format!!!\033[0m" << endl;
  }
}
