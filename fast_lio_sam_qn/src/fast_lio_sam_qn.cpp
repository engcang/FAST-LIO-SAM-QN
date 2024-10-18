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
  m_nh.param<int>("/keyframe/num_submap_keyframes", lc_config_.num_submap_keyframes_, 5);
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
  m_sub_save_flag = m_nh.subscribe("/save_dir", 1, &FastLioSamQn::saveFlagCallback, this);
  // Timers at the end
  m_loop_timer = m_nh.createTimer(ros::Duration(1/loop_update_hz_), &FastLioSamQn::loopTimerFunc, this);
  m_vis_timer = m_nh.createTimer(ros::Duration(1/vis_hz_), &FastLioSamQn::visTimerFunc, this);

  ROS_WARN("Main class, starting node...");
}

void FastLioSamQn::odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
  Eigen::Matrix4d last_odom_tf_;
  last_odom_tf_ = m_current_frame.pose_eig; //to calculate delta
  m_current_frame = PosePcd(*odom_msg, *pcd_msg, m_current_keyframe_idx); //to be checked if keyframe or not

  if (!m_init) //// init only once
  {
    //others
    m_keyframes.push_back(m_current_frame);
    updateVisVars(m_current_frame);
    m_corrected_current_pcd_pub.publish(pclToPclRos(m_current_frame.pcd, m_map_frame));
    //graph
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished()); // rad*rad, meter*meter
    m_gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseEigToGtsamPose(m_current_frame.pose_eig), prior_noise_));
    m_init_esti.insert(m_current_keyframe_idx, poseEigToGtsamPose(m_current_frame.pose_eig));
    {
      std::lock_guard<std::mutex> lock(m_realtime_pose_mutex);
      m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_frame.pose_eig;
      m_current_frame.pose_corrected_eig = m_last_corrected_pose * m_odom_delta;
      geometry_msgs::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(m_current_frame.pose_corrected_eig, m_map_frame);
      m_realtime_pose_pub.publish(current_pose_stamped_);
      tf::Transform transform_;
      transform_.setOrigin(tf::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
      transform_.setRotation(tf::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
      m_broadcaster.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), m_map_frame, "robot"));
    }
    m_current_keyframe_idx++;
    m_init = true;
  }
  else
  {
    //// 1. realtime pose = last corrected odom * delta (last -> current)
    high_resolution_clock::time_point t1_ = high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> lock(m_realtime_pose_mutex);
      m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_frame.pose_eig;
      m_current_frame.pose_corrected_eig = m_last_corrected_pose * m_odom_delta;
      geometry_msgs::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(m_current_frame.pose_corrected_eig, m_map_frame);
      m_realtime_pose_pub.publish(current_pose_stamped_);
      tf::Transform transform_;
      transform_.setOrigin(tf::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
      transform_.setRotation(tf::Quaternion(current_pose_stamped_.pose.orientation.x,
                                            current_pose_stamped_.pose.orientation.y,
                                            current_pose_stamped_.pose.orientation.z,
                                            current_pose_stamped_.pose.orientation.w));
      m_broadcaster.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), m_map_frame, "robot"));
    }
    // pub current scan in corrected pose frame
    m_corrected_current_pcd_pub.publish(pclToPclRos(transformPcd(m_current_frame.pcd, m_current_frame.pose_corrected_eig), m_map_frame));

    //// 2. check if keyframe
    high_resolution_clock::time_point t2_ = high_resolution_clock::now();
    if (checkIfKeyframe(m_current_frame, m_keyframes.back()))
    {
      // 2-2. if so, save
      {
        std::lock_guard<std::mutex> lock(m_keyframes_mutex);
        m_keyframes.push_back(m_current_frame);
        m_not_processed_keyframe = m_current_frame; //to check loop in another thread        
      }
      // 2-3. if so, add to graph
      gtsam::noiseModel::Diagonal::shared_ptr odom_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6)
                                                                                                    << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
      gtsam::Pose3 pose_from_ = poseEigToGtsamPose(m_keyframes[m_current_keyframe_idx-1].pose_corrected_eig);
      gtsam::Pose3 pose_to_ = poseEigToGtsamPose(m_current_frame.pose_corrected_eig);
      {
        std::lock_guard<std::mutex> lock(m_graph_mutex);
        m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(m_current_keyframe_idx-1, m_current_keyframe_idx, pose_from_.between(pose_to_), odom_noise_));
        m_init_esti.insert(m_current_keyframe_idx, pose_to_);
      }
      m_current_keyframe_idx++;

      //// 3. vis
      high_resolution_clock::time_point t3_ = high_resolution_clock::now();
      {
        std::lock_guard<std::mutex> lock(m_vis_mutex);
        updateVisVars(m_current_frame);
      }

      //// 4. optimize with graph
      high_resolution_clock::time_point t4_ = high_resolution_clock::now();
      // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, m_init_esti).optimize(); // cf. isam.update vs values.LM.optimize
      {
        std::lock_guard<std::mutex> lock(m_graph_mutex);
        m_isam_handler->update(m_gtsam_graph, m_init_esti);
        m_isam_handler->update();
        if (m_loop_added_flag) //https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
        {
          m_isam_handler->update();
          m_isam_handler->update();
          m_isam_handler->update();
        }
        m_gtsam_graph.resize(0);
        m_init_esti.clear();
      }

      //// 5. handle corrected results
      // get corrected poses and reset odom delta (for realtime pose pub)
      high_resolution_clock::time_point t5_ = high_resolution_clock::now();
      {
        std::lock_guard<std::mutex> lock(m_realtime_pose_mutex);
        m_corrected_esti = m_isam_handler->calculateEstimate();
        m_last_corrected_pose = gtsamPoseToPoseEig(m_corrected_esti.at<gtsam::Pose3>(m_corrected_esti.size()-1));
        m_odom_delta = Eigen::Matrix4d::Identity();
      }
      // correct poses in keyframes
      if (m_loop_added_flag)
      {
        std::lock_guard<std::mutex> lock(m_keyframes_mutex);
        for (size_t i = 0; i < m_corrected_esti.size(); ++i)
        {
          m_keyframes[i].pose_corrected_eig = gtsamPoseToPoseEig(m_corrected_esti.at<gtsam::Pose3>(i));
        }
        m_loop_added_flag = false;
      }
      high_resolution_clock::time_point t6_ = high_resolution_clock::now();

      ROS_INFO("real: %.1f, key_add: %.1f, vis: %.1f, opt: %.1f, res: %.1f, tot: %.1fms", 
        duration_cast<microseconds>(t2_-t1_).count()/1e3, duration_cast<microseconds>(t3_-t2_).count()/1e3,
        duration_cast<microseconds>(t4_-t3_).count()/1e3, duration_cast<microseconds>(t5_-t4_).count()/1e3,
        duration_cast<microseconds>(t6_-t5_).count()/1e3, duration_cast<microseconds>(t6_-t1_).count()/1e3);
    }
  }
  return;
}

void FastLioSamQn::loopTimerFunc(const ros::TimerEvent& event)
{
  if (!m_init) return;

  //// 1. copy keyframes and not processed keyframes
  high_resolution_clock::time_point t1_ = high_resolution_clock::now();
  PosePcd not_proc_key_copy_;
  std::vector<PosePcd> keyframes_copy_;
  {
    std::lock_guard<std::mutex> lock(m_keyframes_mutex);
    if (!m_not_processed_keyframe.processed)
    {
      not_proc_key_copy_ = m_not_processed_keyframe;
      m_not_processed_keyframe.processed = true;
      keyframes_copy_ = m_keyframes;
    }
  }
  if (not_proc_key_copy_.idx == 0 || not_proc_key_copy_.processed || keyframes_copy_.empty()) return; //already processed

  //// 2. detect loop and add to graph
  high_resolution_clock::time_point t2_ = high_resolution_clock::now();
  const RegistrationOutput &reg_output = loop_closure_->retrieveLoopClosureMeasurement(not_proc_key_copy_, keyframes_copy_);
  const int closest_keyframe_idx = loop_closure_->getClosestKeyframeidx();

  if (closest_keyframe_idx < 0) return;

  if (reg_output.is_valid_)
  {
    ROS_INFO("\033[1;32mLoop closure accepted. Score: %.3f", reg_output.score_, "\033[0m");
    const auto &score_ = reg_output.score_;
    gtsam::Pose3 pose_from_ = poseEigToGtsamPose(reg_output.pose_between_eig_ * not_proc_key_copy_.pose_corrected_eig); //IMPORTANT: take care of the order
    gtsam::Pose3 pose_to_ = poseEigToGtsamPose(keyframes_copy_[closest_keyframe_idx].pose_corrected_eig);
    gtsam::noiseModel::Diagonal::shared_ptr loop_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << score_, score_, score_, score_, score_, score_).finished());
    {
      std::lock_guard<std::mutex> lock(m_graph_mutex);
      m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(not_proc_key_copy_.idx, closest_keyframe_idx, pose_from_.between(pose_to_), loop_noise_));
    }
    m_loop_idx_pairs.push_back({not_proc_key_copy_.idx, closest_keyframe_idx}); //for vis

    m_loop_added_flag_vis = true;
    m_loop_added_flag = true;
  }
  else 
  { 
      ROS_WARN("Loop closure rejected. Score: %.3f", reg_output.score_);
  }
  
  high_resolution_clock::time_point t3_ = high_resolution_clock::now();

  m_debug_src_pub.publish(pclToPclRos(loop_closure_->getSourceCloud(), m_map_frame));
  m_debug_dst_pub.publish(pclToPclRos(loop_closure_->getTargetCloud(), m_map_frame));
  m_debug_fine_aligned_pub.publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), m_map_frame));
  m_debug_coarse_aligned_pub.publish(pclToPclRos(loop_closure_->getCoarseAlignedCloud(), m_map_frame));

  ROS_INFO("copy: %.1f, loop: %.1f", duration_cast<microseconds>(t2_-t1_).count()/1e3, duration_cast<microseconds>(t3_-t2_).count()/1e3);
  return;
}

void FastLioSamQn::visTimerFunc(const ros::TimerEvent& event)
{
  if (!m_init) return;

  high_resolution_clock::time_point tv1_ = high_resolution_clock::now();
  //// 1. if loop closed, correct vis data
  if (m_loop_added_flag_vis) 
  {
    // copy and ready
    gtsam::Values corrected_esti_copy_;
    pcl::PointCloud<pcl::PointXYZ> corrected_odoms_;
    nav_msgs::Path corrected_path_;
    {
      std::lock_guard<std::mutex> lock(m_realtime_pose_mutex);
      corrected_esti_copy_ = m_corrected_esti;
    }
    // correct pose and path
    for (size_t i = 0; i < corrected_esti_copy_.size(); ++i)
    {
      gtsam::Pose3 pose_ = corrected_esti_copy_.at<gtsam::Pose3>(i);
      corrected_odoms_.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
      corrected_path_.poses.push_back(gtsamPoseToPoseStamped(pose_, m_map_frame));
    }
    // update vis of loop constraints
    if (!m_loop_idx_pairs.empty())
    {
      m_loop_detection_pub.publish(getLoopMarkers(corrected_esti_copy_));
    }
    // update with corrected data
    {
      std::lock_guard<std::mutex> lock(m_vis_mutex);
      m_corrected_odoms = corrected_odoms_;
      m_corrected_path.poses = corrected_path_.poses;
    }
    m_loop_added_flag_vis = false;
  }

  //// 2. publish odoms, paths
  {
    std::lock_guard<std::mutex> lock(m_vis_mutex);
    m_odom_pub.publish(pclToPclRos(m_odoms, m_map_frame));
    m_path_pub.publish(m_odom_path);
    m_corrected_odom_pub.publish(pclToPclRos(m_corrected_odoms, m_map_frame));
    m_corrected_path_pub.publish(m_corrected_path);
  }

  //// 3. global map
  if (m_global_map_vis_switch && m_corrected_pcd_map_pub.getNumSubscribers() > 0) //save time, only once
  {
    pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
    {
      std::lock_guard<std::mutex> lock(m_keyframes_mutex);
      for (size_t i = 0; i < m_keyframes.size(); ++i)
      {
        *corrected_map += transformPcd(m_keyframes[i].pcd, m_keyframes[i].pose_corrected_eig);
      }
    }
    const auto &voxelized_map = voxelizePcd(corrected_map, m_voxel_res);
    m_corrected_pcd_map_pub.publish(pclToPclRos(*voxelized_map, m_map_frame));
    m_global_map_vis_switch = false;
  }
  if (!m_global_map_vis_switch && m_corrected_pcd_map_pub.getNumSubscribers() == 0)
  {
    m_global_map_vis_switch = true;      
  }
  high_resolution_clock::time_point tv2_ = high_resolution_clock::now();
  ROS_INFO("vis: %.1fms", duration_cast<microseconds>(tv2_-tv1_).count()/1e3);
  return;
}

void FastLioSamQn::saveFlagCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string save_dir_ = msg->data != "" ? msg->data : m_package_path;

  // save scans as individual pcd files and poses in KITTI format
  // Delete the scans folder if it exists and create a new one
  std::string seq_directory_   = save_dir_ + "/" + m_seq_name;
  std::string scans_directory_ = seq_directory_ + "/scans";
  if (m_save_in_kitti_format)
  {
    std::cout << "\033[32;1mScans are saved in " << scans_directory_ << ", following the KITTI and TUM format\033[0m" << std::endl;
    if (fs::exists(seq_directory_))
    {
      fs::remove_all(seq_directory_);
    }
    fs::create_directories(scans_directory_);

    std::ofstream kitti_pose_file_(seq_directory_ + "/poses_kitti.txt");
    std::ofstream tum_pose_file_(seq_directory_ + "/poses_tum.txt");
    tum_pose_file_ << "#timestamp x y z qx qy qz qw\n";
    {
      std::lock_guard<std::mutex> lock(m_keyframes_mutex);
      for (size_t i = 0; i < m_keyframes.size(); ++i)
      {
        // Save the point cloud
        std::stringstream ss_;
        ss_ << scans_directory_ << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
        std::cout << "Saving " << ss_.str()  << "..." << std::endl;
        pcl::io::savePCDFileASCII<PointType>(ss_.str(), m_keyframes[i].pcd);

        // Save the pose in KITTI format
        const auto &pose_ = m_keyframes[i].pose_corrected_eig;
        kitti_pose_file_ << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " " << pose_(0, 3) << " "
                        << pose_(1, 0) << " " << pose_(1, 1) << " " << pose_(1, 2) << " " << pose_(1, 3) << " "
                        << pose_(2, 0) << " " << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

        const auto &lidar_optim_pose_ = poseEigToPoseStamped(m_keyframes[i].pose_corrected_eig);
        tum_pose_file_ << std::fixed << std::setprecision(8)
                      << m_keyframes[i].timestamp << " "
                      << lidar_optim_pose_.pose.position.x << " "
                      << lidar_optim_pose_.pose.position.y << " "
                      << lidar_optim_pose_.pose.position.z << " "
                      << lidar_optim_pose_.pose.orientation.x << " "
                      << lidar_optim_pose_.pose.orientation.y << " "
                      << lidar_optim_pose_.pose.orientation.z << " "
                      << lidar_optim_pose_.pose.orientation.w << "\n";
      }
    }
    kitti_pose_file_.close();
    tum_pose_file_.close();
    std::cout << "\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m" << std::endl;
  }

  if (m_save_map_bag)
  {
    rosbag::Bag bag_;
    bag_.open(m_package_path+"/result.bag", rosbag::bagmode::Write);
    {
      std::lock_guard<std::mutex> lock(m_keyframes_mutex);
      for (size_t i = 0; i < m_keyframes.size(); ++i)
      {
        ros::Time time_;
        time_.fromSec(m_keyframes[i].timestamp);
        bag_.write("/keyframe_pcd", time_, pclToPclRos(m_keyframes[i].pcd, m_map_frame));
        bag_.write("/keyframe_pose", time_, poseEigToPoseStamped(m_keyframes[i].pose_corrected_eig));
      }
    }
    bag_.close();
    std::cout << "\033[36;1mResult saved in .bag format!!!\033[0m" << std::endl;
  }

  if (m_save_map_pcd)
  {
    pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
    {
      std::lock_guard<std::mutex> lock(m_keyframes_mutex);
      for (size_t i = 0; i < m_keyframes.size(); ++i)
      {
        *corrected_map += transformPcd(m_keyframes[i].pcd, m_keyframes[i].pose_corrected_eig);
      }
    }
    const auto &voxelized_map = voxelizePcd(corrected_map, m_voxel_res);
    pcl::io::savePCDFileASCII<PointType> (seq_directory_ + "/" + m_seq_name + "_map.pcd", *voxelized_map);
    cout << "\033[32;1mAccumulated map cloud saved in .pcd format\033[0m" << endl;
  }
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

void FastLioSamQn::updateVisVars(const PosePcd &pose_pcd_in)
{
  m_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  m_corrected_odoms.points.emplace_back(pose_pcd_in.pose_corrected_eig(0, 3), pose_pcd_in.pose_corrected_eig(1, 3), pose_pcd_in.pose_corrected_eig(2, 3));
  m_odom_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_eig, m_map_frame));
  m_corrected_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig, m_map_frame));
  return;
}

visualization_msgs::Marker FastLioSamQn::getLoopMarkers(const gtsam::Values &corrected_esti_in)
{
  visualization_msgs::Marker edges_; edges_.type = 5u;
  edges_.scale.x = 0.12f; edges_.header.frame_id = m_map_frame; edges_.pose.orientation.w = 1.0f;
  edges_.color.r = 1.0f; edges_.color.g = 1.0f; edges_.color.b = 1.0f; edges_.color.a = 1.0f;
  for (int i = 0; i < m_loop_idx_pairs.size(); ++i)
  {
    if (m_loop_idx_pairs[i].first >= corrected_esti_in.size() || m_loop_idx_pairs[i].second >= corrected_esti_in.size()) continue;
    gtsam::Pose3 pose_ = corrected_esti_in.at<gtsam::Pose3>(m_loop_idx_pairs[i].first);
    gtsam::Pose3 pose2_ = corrected_esti_in.at<gtsam::Pose3>(m_loop_idx_pairs[i].second);
    geometry_msgs::Point p_, p2_;
    p_.x = pose_.translation().x(); p_.y = pose_.translation().y(); p_.z = pose_.translation().z();
    p2_.x = pose2_.translation().x(); p2_.y = pose2_.translation().y(); p2_.z = pose2_.translation().z();
    edges_.points.push_back(p_);
    edges_.points.push_back(p2_);
  }
  return edges_;
}

bool FastLioSamQn::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
  return m_keyframe_thr < (latest_pose_pcd.pose_corrected_eig.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig.block<3, 1>(0, 3)).norm();
}


