#include "fast_lio_sam_qn.h"

namespace fs = std::filesystem;

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
      transform_.setRotation(tf::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
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
      gtsam::noiseModel::Diagonal::shared_ptr odom_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
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
        for (int i = 0; i < m_corrected_esti.size(); ++i)
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
  bool if_loop_occured_ = false;
  // from not_proc_key_copy_ keyframe to old keyframes in threshold radius, get the closest keyframe
  int closest_keyframe_idx_ = getClosestKeyframeIdx(not_proc_key_copy_, keyframes_copy_);
  if (closest_keyframe_idx_ >= 0) //if exists
  {
    // Quatro + NANO-GICP to check loop (from front_keyframe to closest keyframe's neighbor)
    RegistrationOutput reg_output;
    const auto &[src_raw, dst_raw] = setSrcAndDstCloud(m_keyframes, not_proc_key_copy_.idx, closest_keyframe_idx_, m_sub_key_num, m_enable_quatro, m_enable_submap_matching);
    if (m_enable_quatro) 
    {
      ROS_INFO("\033[1;35mcoarseToFineKeyToKey\033[0m");
      reg_output = coarseToFineAlignment(src_raw, dst_raw);
    }
    else
    {
      reg_output = icpAlignment(src_raw, dst_raw);
    }

    if(reg_output.is_converged) // add loop factor
    {
      ROS_INFO("\033[1;32mLoop closure accepted. Score: %.3f", reg_output.score, "\033[0m");
      const auto &score_ = reg_output.score;
      gtsam::Pose3 pose_from_ = poseEigToGtsamPose(reg_output.pose_between_eig * not_proc_key_copy_.pose_corrected_eig); //IMPORTANT: take care of the order
      gtsam::Pose3 pose_to_ = poseEigToGtsamPose(keyframes_copy_[closest_keyframe_idx_].pose_corrected_eig);
      gtsam::noiseModel::Diagonal::shared_ptr loop_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << score_, score_, score_, score_, score_, score_).finished());
      {
        std::lock_guard<std::mutex> lock(m_graph_mutex);
        m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(not_proc_key_copy_.idx, closest_keyframe_idx_, pose_from_.between(pose_to_), loop_noise_));
      }
      m_loop_idx_pairs.push_back({not_proc_key_copy_.idx, closest_keyframe_idx_}); //for vis
      if_loop_occured_ = true;
    }
    else 
    {
      ROS_WARN("Loop closure rejected. Score: %.3f", reg_output.score);
    }
  }
  high_resolution_clock::time_point t3_ = high_resolution_clock::now();

  if (if_loop_occured_)
  {
    m_loop_added_flag_vis = true;
    m_loop_added_flag = true;
  }

  ROS_INFO("copy: %.1f, loop: %.1f", 
          duration_cast<microseconds>(t2_-t1_).count()/1e3, duration_cast<microseconds>(t3_-t2_).count()/1e3);

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
    for (int i = 0; i < corrected_esti_copy_.size(); ++i)
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
      for (int i = 0; i < m_keyframes.size(); ++i)
      {
        *corrected_map += transformPcd(m_keyframes[i].pcd, m_keyframes[i].pose_corrected_eig);
      }
    }
    const auto voxelized_map = voxelizePcd(corrected_map, m_voxel_res);
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

void FastLioSamQn::SaveFlagCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string save_dir = msg->data != "" ? msg->data : m_package_path;

  // save scans as individual pcd files and poses in KITTI format
  // Delete the scans folder if it exists and create a new one
  std::string seq_directory   = save_dir + "/" + m_seq_name;
  std::string scans_directory = seq_directory + "/scans";
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
    std::lock_guard<std::mutex> lock(m_keyframes_mutex);
    for (size_t i = 0; i < m_keyframes.size(); ++i)
    {
      // Save the point cloud
      std::stringstream ss;
      ss << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
      std::cout << "Saving " << ss.str()  << "..." << std::endl;
      pcl::io::savePCDFileASCII<PointType>(ss.str(), m_keyframes[i].pcd);

      // Save the pose in KITTI format
      const auto &pose = m_keyframes[i].pose_corrected_eig;
      kitti_pose_file << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " " << pose(0, 3) << " "
                << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " " << pose(1, 3) << " "
                << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << "\n";

      const auto &lidar_optim_pose = poseEigToPoseStamped(m_keyframes[i].pose_corrected_eig);
      tum_pose_file << std::fixed << std::setprecision(8)
              << m_keyframes[i].timestamp << " "
              << lidar_optim_pose.pose.position.x << " "
              << lidar_optim_pose.pose.position.y << " "
              << lidar_optim_pose.pose.position.z << " "
              << lidar_optim_pose.pose.orientation.x << " "
              << lidar_optim_pose.pose.orientation.y << " "
              << lidar_optim_pose.pose.orientation.z << " "
              << lidar_optim_pose.pose.orientation.w << "\n";
    }
  }
  kitti_pose_file.close();
  tum_pose_file.close();
  std::cout << "\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m" << std::endl;

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
    pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
    {
      std::lock_guard<std::mutex> lock(m_keyframes_mutex);
      for (size_t i = 0; i < m_keyframes.size(); ++i)
      {
        *corrected_map += transformPcd(m_keyframes[i].pcd, m_keyframes[i].pose_corrected_eig);
      }
    }
    const auto voxelized_map = voxelizePcd(corrected_map, m_voxel_res);
    pcl::io::savePCDFileASCII<PointType> (seq_directory + "/" + m_seq_name + "_map.pcd", *voxelized_map);
    cout << "\033[32;1mAccumulated map cloud saved in .pcd format\033[0m" << endl;
  }
}
