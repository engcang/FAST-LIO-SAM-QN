#include "fast_lio_sam_qn.h"

FastLioSamQn::FastLioSamQn(const ros::NodeHandle& n_private) : nh_(n_private)
{
  ////// ROS params
  double loop_update_hz, vis_hz;
  auto & gc = lc_config_.gicp_config_;
  auto & qc = lc_config_.quatro_config_;
  /* basic */
  nh_.param<std::string>("/basic/map_frame", map_frame_, "map");
  nh_.param<double>("/basic/loop_update_hz", loop_update_hz, 1.0);
  nh_.param<double>("/basic/vis_hz", vis_hz, 0.5);
  nh_.param<double>("/save_voxel_resolution", voxel_res_, 0.3);
  nh_.param<double>("/quatro_nano_gicp_voxel_resolution", lc_config_.voxel_res_, 0.3);
  /* keyframe */
  nh_.param<double>("/keyframe/keyframe_threshold", keyframe_thr_, 1.0);
  nh_.param<int>("/keyframe/nusubmap_keyframes", lc_config_.num_submap_keyframes_, 5);
  nh_.param<bool>("/keyframe/enable_submap_matching", lc_config_.enable_submap_matching_, false);
  /* loop */
  nh_.param<double>("/loop/loop_detection_radius", lc_config_.loop_detection_radius_, 15.0);
  nh_.param<double>("/loop/loop_detection_timediff_threshold", lc_config_.loop_detection_timediff_threshold_, 10.0);
  /* nano (GICP config) */
  nh_.param<int>("/nano_gicp/thread_number", gc.nano_thread_number_, 0);
  nh_.param<double>("/nano_gicp/icp_score_threshold", gc.icp_score_thr_, 10.0);
  nh_.param<int>("/nano_gicp/correspondences_number", gc.nano_correspondences_number_, 15);
  nh_.param<int>("/nano_gicp/max_iter", gc.nano_max_iter_, 32);
  nh_.param<double>("/nano_gicp/transformation_epsilon", gc.transformation_epsilon_, 0.01);
  nh_.param<double>("/nano_gicp/euclidean_fitness_epsilon", gc.euclidean_fitness_epsilon_, 0.01);
  nh_.param<int>("/nano_gicp/ransac/max_iter", gc.nano_ransac_max_iter_, 5);
  nh_.param<double>("/nano_gicp/ransac/outlier_rejection_threshold", gc.ransac_outlier_rejection_threshold_, 1.0);
  /* quatro (Quatro config) */
  nh_.param<bool>("/quatro/enable", lc_config_.enable_quatro_, false);
  nh_.param<bool>("/quatro/optimize_matching", qc.use_optimized_matching_, true);
  nh_.param<double>("/quatro/distance_threshold", qc.quatro_distance_threshold_, 30.0);
  nh_.param<int>("/quatro/max_nucorrespondences", qc.quatro_max_num_corres_, 200);
  nh_.param<double>("/quatro/fpfh_normal_radius", qc.fpfh_normal_radius_, 0.3);
  nh_.param<double>("/quatro/fpfh_radius", qc.fpfh_radius_, 0.5);
  nh_.param<bool>("/quatro/estimating_scale", qc.estimat_scale_, false);
  nh_.param<double>("/quatro/noise_bound", qc.noise_bound_, 0.3);
  nh_.param<double>("/quatro/rotation/gnc_factor", qc.rot_gnc_factor_, 1.4);
  nh_.param<double>("/quatro/rotation/rot_cost_diff_threshold", qc.rot_cost_diff_thr_, 0.0001);
  nh_.param<int>("/quatro/rotation/numax_iter", qc.quatro_max_iter_, 50);
  /* results */
  nh_.param<bool>("/result/save_map_bag", save_map_bag_, false);
  nh_.param<bool>("/result/save_map_pcd", save_map_pcd_, false);
  nh_.param<bool>("/result/save_in_kitti_format", save_in_kitti_format_, false);
  nh_.param<std::string>("/result/seq_name", seq_name_, "");
  loop_closure_.reset(new LoopClosure(lc_config_));
  /* Initialization of GTSAM */
  gtsam::ISAM2Params isam_params_;
  isam_params_.relinearizeThreshold = 0.01;
  isam_params_.relinearizeSkip = 1;
  isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);
  /* ROS things */
  odom_path_.header.frame_id = map_frame_;
  corrected_path_.header.frame_id = map_frame_;
  package_path_ = ros::package::getPath("fast_lio_saqn");
  /* publishers */
  odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ori_odom", 10, true);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/ori_path", 10, true);
  corrected_odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_odom", 10, true);
  corrected_path_pub_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10, true);
  corrected_pcd_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_map", 10, true);
  corrected_current_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_current_pcd", 10, true);
  loop_detection_pub_ = nh_.advertise<visualization_msgs::Marker>("/loop_detection", 10, true);
  realtime_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_stamped", 10);
  debug_src_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/src", 10, true);
  debug_dst_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dst", 10, true);
  debug_coarse_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/coarse_aligned_quatro", 10, true);
  debug_fine_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fine_aligned_nano_gicp", 10, true);
  /* subscribers */
  sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, "/Odometry", 10);
  sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, "/cloud_registered", 10);
  sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
  sub_odom_pcd_sync_->registerCallback(boost::bind(&FastLioSamQn::odomPcdCallback, this, _1, _2));
  sub_save_flag_ = nh_.subscribe("/save_dir", 1, &FastLioSamQn::saveFlagCallback, this);
  /* Timers */
  loop_timer_ = nh_.createTimer(ros::Duration(1/loop_update_hz), &FastLioSamQn::loopTimerFunc, this);
  vis_timer_ = nh_.createTimer(ros::Duration(1/vis_hz), &FastLioSamQn::visTimerFunc, this);
  ROS_INFO("Main class, starting node...");
}

void FastLioSamQn::odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
  Eigen::Matrix4d last_odom_tf;
  last_odom_tf = current_frame_.pose_eig_; //to calculate delta
  current_frame_ = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); //to be checked if keyframe or not

  if (!init_) //// init only once
  {
    //others
    keyframes_.push_back(current_frame_);
    updateVisVars(current_frame_);
    corrected_current_pcd_pub_.publish(pclToPclRos(current_frame_.pcd_, map_frame_));
    //graph
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished()); // rad*rad, meter*meter
    gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
    init_esti_.insert(current_keyframe_idx_, poseEigToGtsamPose(current_frame_.pose_eig_));
    {
      std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
      odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
      current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
      geometry_msgs::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_);
      realtime_pose_pub_.publish(current_pose_stamped_);
      tf::Transform transform_;
      transform_.setOrigin(tf::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
      transform_.setRotation(tf::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
      broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), map_frame_, "robot"));
    }
    current_keyframe_idx_++;
    init_ = true;
  }
  else
  {
    //// 1. realtime pose = last corrected odom * delta (last -> current)
    high_resolution_clock::time_point t1_ = high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
      odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
      current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
      geometry_msgs::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_);
      realtime_pose_pub_.publish(current_pose_stamped_);
      tf::Transform transform_;
      transform_.setOrigin(tf::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
      transform_.setRotation(tf::Quaternion(current_pose_stamped_.pose.orientation.x,
                                            current_pose_stamped_.pose.orientation.y,
                                            current_pose_stamped_.pose.orientation.z,
                                            current_pose_stamped_.pose.orientation.w));
      broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), map_frame_, "robot"));
    }
    // pub current scan in corrected pose frame
    corrected_current_pcd_pub_.publish(pclToPclRos(transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_), map_frame_));

    //// 2. check if keyframe
    high_resolution_clock::time_point t2_ = high_resolution_clock::now();
    if (checkIfKeyframe(current_frame_, keyframes_.back()))
    {
      // 2-2. if so, save
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        keyframes_.push_back(current_frame_);
        not_processed_keyframe_ = current_frame_; //to check loop in another thread        
      }
      // 2-3. if so, add to graph
      gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6)
                                                                                                    << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
      gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[current_keyframe_idx_-1].pose_corrected_eig_);
      gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_-1, current_keyframe_idx_, pose_from.between(pose_to), odom_noise));
        init_esti_.insert(current_keyframe_idx_, pose_to);
      }
      current_keyframe_idx_++;

      //// 3. vis
      high_resolution_clock::time_point t3_ = high_resolution_clock::now();
      {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        updateVisVars(current_frame_);
      }

      //// 4. optimize with graph
      high_resolution_clock::time_point t4_ = high_resolution_clock::now();
      // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, init_esti_).optimize(); // cf. isam.update vs values.LM.optimize
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        isam_handler_->update(gtsam_graph_, init_esti_);
        isam_handler_->update();
        if (loop_added_flag_) //https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
        {
          isam_handler_->update();
          isam_handler_->update();
          isam_handler_->update();
        }
        gtsam_graph_.resize(0);
        init_esti_.clear();
      }

      //// 5. handle corrected results
      // get corrected poses and reset odom delta (for realtime pose pub)
      high_resolution_clock::time_point t5_ = high_resolution_clock::now();
      {
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        corrected_esti_ = isam_handler_->calculateEstimate();
        last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size()-1));
        odom_delta_ = Eigen::Matrix4d::Identity();
      }
      // correct poses in keyframes
      if (loop_added_flag_)
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        for (size_t i = 0; i < corrected_esti_.size(); ++i)
        {
          keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
        }
        loop_added_flag_ = false;
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
  if (!init_) return;

  //// 1. copy keyframes and not processed keyframes
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  PosePcd not_proc_key_copied;
  std::vector<PosePcd> keyframes_copied;
  {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    if (!not_processed_keyframe_.processed_)
    {
      not_proc_key_copied = not_processed_keyframe_;
      not_processed_keyframe_.processed_ = true;
      keyframes_copied = keyframes_;
    }
  }
  if (not_proc_key_copied.idx_ == 0 || not_proc_key_copied.processed_ || keyframes_copied.empty()) return; //already processed

  //// 2. detect loop and add to graph
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  const int closest_keyframe_idx = loop_closure_->fetchClosestKeyframeIdx(not_proc_key_copied, keyframes_copied);
  if (closest_keyframe_idx < 0) return;

  const RegistrationOutput &reg_output = loop_closure_->performLoopClosure(not_proc_key_copied, keyframes_copied, closest_keyframe_idx);
  if (reg_output.is_valid_)
  {
    ROS_INFO("\033[1;32mLoop closure accepted. Score: %.3f", reg_output.score_, "\033[0m");
    const auto &score = reg_output.score_;
    gtsam::Pose3 pose_from = poseEigToGtsamPose(reg_output.pose_between_eig_ * not_proc_key_copied.pose_corrected_eig_); //IMPORTANT: take care of the order
    gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_copied[closest_keyframe_idx].pose_corrected_eig_);
    gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << score, score, score, score, score, score).finished());
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(not_proc_key_copied.idx_, closest_keyframe_idx, pose_from.between(pose_to), loop_noise));
    }
    loop_idx_pairs_.push_back({not_proc_key_copied.idx_, closest_keyframe_idx}); //for vis

    loop_added_flag_vis_ = true;
    loop_added_flag_ = true;
  }
  else 
  { 
      ROS_WARN("Loop closure rejected. Score: %.3f", reg_output.score_);
  }
  
  high_resolution_clock::time_point t3 = high_resolution_clock::now();

  debug_src_pub_.publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
  debug_dst_pub_.publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
  debug_fine_aligned_pub_.publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));
  debug_coarse_aligned_pub_.publish(pclToPclRos(loop_closure_->getCoarseAlignedCloud(), map_frame_));

  ROS_INFO("copy: %.1f, loop: %.1f", duration_cast<microseconds>(t2 - t1).count() / 1e3, duration_cast<microseconds>(t3 - t2).count() / 1e3);
  return;
}

void FastLioSamQn::visTimerFunc(const ros::TimerEvent& event)
{
  if (!init_) return;

  high_resolution_clock::time_point tv1 = high_resolution_clock::now();
  //// 1. if loop closed, correct vis data
  if (loop_added_flag_vis_) 
  // copy and ready
  {
    gtsam::Values corrected_esti_copied;
    pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
    nav_msgs::Path corrected_path;
    {
      std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
      corrected_esti_copied = corrected_esti_;
    }
    // correct pose and path
    for (size_t i = 0; i < corrected_esti_copied.size(); ++i)
    {
      gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
      corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
      corrected_path.poses.push_back(gtsamPoseToPoseStamped(pose_, map_frame_));
    }
    // update vis of loop constraints
    if (!loop_idx_pairs_.empty())
    {
      loop_detection_pub_.publish(getLoopMarkers(corrected_esti_copied));
    }
    // update with corrected data
    {
      std::lock_guard<std::mutex> lock(vis_mutex_);
      corrected_odoms_ = corrected_odoms;
      corrected_path_.poses = corrected_path.poses;
    }
    loop_added_flag_vis_ = false;
  }
  //// 2. publish odoms, paths
  {
    std::lock_guard<std::mutex> lock(vis_mutex_);
    odom_pub_.publish(pclToPclRos(odoms_, map_frame_));
    path_pub_.publish(odom_path_);
    corrected_odom_pub_.publish(pclToPclRos(corrected_odoms_, map_frame_));
    corrected_path_pub_.publish(corrected_path_);
  }

  //// 3. global map
  if (global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() > 0) //save time, only once
  {
    pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i)
      {
        *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
      }
    }
    const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
    corrected_pcd_map_pub_.publish(pclToPclRos(*voxelized_map, map_frame_));
    global_map_vis_switch_ = false;
  }
  if (!global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() == 0)
  {
    global_map_vis_switch_ = true;      
  }
  high_resolution_clock::time_point tv2 = high_resolution_clock::now();
  ROS_INFO("vis: %.1fms", duration_cast<microseconds>(tv2 - tv1).count()/1e3);
  return;
}

void FastLioSamQn::saveFlagCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string save_dir = msg->data != "" ? msg->data : package_path_;

  // save scans as individual pcd files and poses in KITTI format
  // Delete the scans folder if it exists and create a new one
  std::string seq_directory   = save_dir + "/" + seq_name_;
  std::string scans_directory = seq_directory + "/scans";
  if (save_in_kitti_format_)
  {
    std::cout << "\033[32;1mScans are saved in " << scans_directory << ", following the KITTI and TUM format\033[0m" << std::endl;
    if (fs::exists(seq_directory))
    {
      fs::remove_all(seq_directory);
    }
    fs::create_directories(scans_directory);

    std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
    std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
    tum_pose_file << "#timestamp x y z qx qy qz qw\n";
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i)
      {
        // Save the point cloud
        std::stringstream ss_;
        ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
        std::cout << "Saving " << ss_.str()  << "..." << std::endl;
        pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].pcd_);

        // Save the pose in KITTI format
        const auto &pose_ = keyframes_[i].pose_corrected_eig_;
        kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " " << pose_(0, 3) << " "
                        << pose_(1, 0) << " " << pose_(1, 1) << " " << pose_(1, 2) << " " << pose_(1, 3) << " "
                        << pose_(2, 0) << " " << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

        const auto &lidar_optim_pose_ = poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
        tum_pose_file << std::fixed << std::setprecision(8)
                      << keyframes_[i].timestamp_ << " "
                      << lidar_optim_pose_.pose.position.x << " "
                      << lidar_optim_pose_.pose.position.y << " "
                      << lidar_optim_pose_.pose.position.z << " "
                      << lidar_optim_pose_.pose.orientation.x << " "
                      << lidar_optim_pose_.pose.orientation.y << " "
                      << lidar_optim_pose_.pose.orientation.z << " "
                      << lidar_optim_pose_.pose.orientation.w << "\n";
      }
    }
    kitti_pose_file.close();
    tum_pose_file.close();
    std::cout << "\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m" << std::endl;
  }

  if (save_map_bag_)
  {
    rosbag::Bag bag;
    bag.open(package_path_+"/result.bag", rosbag::bagmode::Write);
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i)
      {
        ros::Time time;
        time.fromSec(keyframes_[i].timestamp_);
        bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
        bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
      }
    }
    bag.close();
    std::cout << "\033[36;1mResult saved in .bag format!!!\033[0m" << std::endl;
  }

  if (save_map_pcd_)
  {
    pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i)
      {
        *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
      }
    }
    const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
    pcl::io::savePCDFileASCII<PointType> (seq_directory + "/" + seq_name_ + "_map.pcd", *voxelized_map);
    cout << "\033[32;1mAccumulated map cloud saved in .pcd format\033[0m" << endl;
  }
}

FastLioSamQn::~FastLioSamQn()
{
  // save map
  if (save_map_bag_)
  {
    rosbag::Bag bag;
    bag.open(package_path_+"/result.bag", rosbag::bagmode::Write);
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (int i = 0; i < keyframes_.size(); ++i)
      {
        ros::Time time;
        time.fromSec(keyframes_[i].timestamp_);
        bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
        bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
      }
    }
    bag.close();
    cout << "\033[36;1mResult saved in .bag format!!!\033[0m" << endl;
  }
  if (save_map_pcd_)
  {
    pcl::PointCloud<PointType> corrected_map;
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (int i = 0; i < keyframes_.size(); ++i)
      {
        corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
      }
    }
    pcl::io::savePCDFileASCII<PointType> (package_path_+"/result.pcd", corrected_map);
    cout << "\033[32;1mResult saved in .pcd format!!!\033[0m" << endl;
  }
}

void FastLioSamQn::updateVisVars(const PosePcd &pose_pcd_in)
{
  odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3), pose_pcd_in.pose_eig_(1, 3), pose_pcd_in.pose_eig_(2, 3));
  corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3), pose_pcd_in.pose_corrected_eig_(1, 3), pose_pcd_in.pose_corrected_eig_(2, 3));
  odom_path_.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
  corrected_path_.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
  return;
}

visualization_msgs::Marker FastLioSamQn::getLoopMarkers(const gtsam::Values &corrected_esti_in)
{
  visualization_msgs::Marker edges; edges.type = 5u;
  edges.scale.x = 0.12f; edges.header.frame_id = map_frame_; edges.pose.orientation.w = 1.0f;
  edges.color.r = 1.0f; edges.color.g = 1.0f; edges.color.b = 1.0f; edges.color.a = 1.0f;
  for (int i = 0; i < loop_idx_pairs_.size(); ++i)
  {
    if (loop_idx_pairs_[i].first >= corrected_esti_in.size() || loop_idx_pairs_[i].second >= corrected_esti_in.size()) continue;
    gtsam::Pose3 pose = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].first);
    gtsam::Pose3 pose2 = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].second);
    geometry_msgs::Point p, p2;
    p.x = pose.translation().x(); p.y = pose.translation().y(); p.z = pose.translation().z();
    p2.x = pose2.translation().x(); p2.y = pose2.translation().y(); p2.z = pose2.translation().z();
    edges.points.push_back(p);
    edges.points.push_back(p2);
  }
  return edges;
}

bool FastLioSamQn::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
  return keyframe_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}


