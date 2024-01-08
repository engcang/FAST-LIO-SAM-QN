#include "main.h"


void FastLioSamQnClass::updateVisVars(const PosePcd &pose_pcd_in)
{
  m_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  m_corrected_odoms.points.emplace_back(pose_pcd_in.pose_corrected_eig(0, 3), pose_pcd_in.pose_corrected_eig(1, 3), pose_pcd_in.pose_corrected_eig(2, 3));
  m_odom_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_eig, m_map_frame));
  m_corrected_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig, m_map_frame));
  return;
}

visualization_msgs::Marker FastLioSamQnClass::getLoopMarkers(const gtsam::Values &corrected_esti_in)
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

void FastLioSamQnClass::voxelizePcd(pcl::VoxelGrid<PointType> &voxelgrid, pcl::PointCloud<PointType> &pcd_in)
{
  pcl::PointCloud<PointType>::Ptr before_(new pcl::PointCloud<PointType>);
  *before_ = pcd_in;
  voxelgrid.setInputCloud(before_);
  voxelgrid.filter(pcd_in);
  return;
}

bool FastLioSamQnClass::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
  return m_keyframe_thr < (latest_pose_pcd.pose_corrected_eig.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig.block<3, 1>(0, 3)).norm();
}

int FastLioSamQnClass::getClosestKeyframeIdx(const PosePcd &front_keyframe, const std::vector<PosePcd> &keyframes)
{
  double shortest_distance_ = m_loop_det_radi*3.0;
  int closest_idx_ = -1;
  for (int idx = 0; idx < keyframes.size()-1; ++idx)
  {
    //check if potential loop: close enough in distance, far enough in time
    double tmp_dist_ = (keyframes[idx].pose_corrected_eig.block<3, 1>(0, 3) - front_keyframe.pose_corrected_eig.block<3, 1>(0, 3)).norm();
    if (m_loop_det_radi > tmp_dist_ && m_loop_det_tdiff_thr < (front_keyframe.timestamp - keyframes[idx].timestamp))
    {
      if (tmp_dist_ < shortest_distance_)
      {
        shortest_distance_ = tmp_dist_;
        closest_idx_ = keyframes[idx].idx;
      }
    }
  }
  return closest_idx_;
}

Eigen::Matrix4d FastLioSamQnClass::icpKeyToSubkeys(const PosePcd &front_keyframe, const int &closest_idx, const std::vector<PosePcd> &keyframes, bool &if_converged, double &score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType> dst_raw_, src_raw_, aligned_;
  src_raw_ = transformPcd(front_keyframe.pcd, front_keyframe.pose_corrected_eig);
  for (int i = closest_idx-m_sub_key_num; i < closest_idx+m_sub_key_num+1; ++i)
  {
    if (i>=0 && i < keyframes.size()-1) //if exists
    {
      dst_raw_ += transformPcd(keyframes[i].pcd, keyframes[i].pose_corrected_eig);
    }
  }
  // voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  // then match with Nano-GICP
  pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
  *dst_ = dst_raw_;
  *src_ = src_raw_;
  m_nano_gicp.setInputSource(src_);
  m_nano_gicp.calculateSourceCovariances();
  m_nano_gicp.setInputTarget(dst_);
  m_nano_gicp.calculateTargetCovariances();
  m_nano_gicp.align(aligned_);
  // vis for debug
  m_debug_src_pub.publish(pclToPclRos(src_raw_, m_map_frame));
  m_debug_dst_pub.publish(pclToPclRos(dst_raw_, m_map_frame));
  m_debug_fine_aligned_pub.publish(pclToPclRos(aligned_, m_map_frame));
  // handle results
  score = m_nano_gicp.getFitnessScore();
  if(m_nano_gicp.hasConverged() && score < m_icp_score_thr) // if matchness score is lower than threshold, (lower is better)
  {
    if_converged = true;
    output_tf_ = m_nano_gicp.getFinalTransformation().cast<double>();
  }
  return output_tf_;
}

Eigen::Matrix4d FastLioSamQnClass::coarseToFineKeyToKey(const PosePcd &front_keyframe, const int &closest_idx, const std::vector<PosePcd> &keyframes, bool &if_converged, double &score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  // Prepare the keyframes
  pcl::PointCloud<PointType> dst_raw_, src_raw_, src_coarse_aligned_, fine_aligned_;
  src_raw_ = transformPcd(front_keyframe.pcd, front_keyframe.pose_corrected_eig);
  dst_raw_ = transformPcd(keyframes[closest_idx].pcd, keyframes[closest_idx].pose_corrected_eig); //Note: Quatro should work on scan-to-scan (keyframe-to-keyframe), not keyframe-to-merged-many-keyframes
  // voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  // then perform Quatro
  Eigen::Matrix4d quatro_tf_ = m_quatro_handler->align(src_raw_, dst_raw_, if_converged);
  if (!if_converged) return quatro_tf_;
  else //if valid,
  {
    // coarse align with the result of Quatro
    src_coarse_aligned_ = transformPcd(src_raw_, quatro_tf_);
    // then match with Nano-GICP
    pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
    *dst_ = dst_raw_;
    *src_ = src_coarse_aligned_;
    m_nano_gicp.setInputSource(src_);
    m_nano_gicp.calculateSourceCovariances();
    m_nano_gicp.setInputTarget(dst_);
    m_nano_gicp.calculateTargetCovariances();
    m_nano_gicp.align(fine_aligned_);
    // handle results
    score = m_nano_gicp.getFitnessScore();
    if(m_nano_gicp.hasConverged() && score < m_icp_score_thr) // if matchness score is lower than threshold, (lower is better)
    {
      if_converged = true;
      Eigen::Matrix4d icp_tf_ = m_nano_gicp.getFinalTransformation().cast<double>();
      output_tf_ = icp_tf_ * quatro_tf_; // IMPORTANT: take care of the order
    }
    else if_converged = false;
    // vis for debug
    m_debug_src_pub.publish(pclToPclRos(src_raw_, m_map_frame));
    m_debug_dst_pub.publish(pclToPclRos(dst_raw_, m_map_frame));
    m_debug_coarse_aligned_pub.publish(pclToPclRos(src_coarse_aligned_, m_map_frame));
    m_debug_fine_aligned_pub.publish(pclToPclRos(fine_aligned_, m_map_frame));
  }

  return output_tf_;
}

Eigen::Matrix4d FastLioSamQnClass::icpSubkeysToSubkeys(const std::deque<PosePcd> &front_keyframes, const int& not_processed_idx,
                                                      const int &closest_idx, const std::vector<PosePcd> &keyframes, bool &if_converged, double &score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  // Prepare the keyframes
  pcl::PointCloud<PointType> dst_raw_, src_raw_, aligned_;
  for (int i = not_processed_idx-m_sub_key_num; i < not_processed_idx+m_sub_key_num+1; ++i)
  {
    if (i>=0 && i < front_keyframes.size()-1) //if exists
    {
      src_raw_ += transformPcd(front_keyframes[i].pcd, front_keyframes[i].pose_corrected_eig);
    }
  }
  for (int i = closest_idx-m_sub_key_num; i < closest_idx+m_sub_key_num+1; ++i)
  {
    if (i>=0 && i < keyframes.size()-1) //if exists
    {
      dst_raw_ += transformPcd(keyframes[i].pcd, keyframes[i].pose_corrected_eig);
    }
  }
  // Voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  // Match with Nano-GICP
  pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
  *dst_ = dst_raw_;
  *src_ = src_raw_;
  m_nano_gicp.setInputSource(src_);
  m_nano_gicp.calculateSourceCovariances();
  m_nano_gicp.setInputTarget(dst_);
  m_nano_gicp.calculateTargetCovariances();
  m_nano_gicp.align(aligned_);
  // vis for debug
  m_debug_src_pub.publish(pclToPclRos(src_raw_, m_map_frame));
  m_debug_dst_pub.publish(pclToPclRos(dst_raw_, m_map_frame));
  m_debug_fine_aligned_pub.publish(pclToPclRos(aligned_, m_map_frame));
  // handle results
  score = m_nano_gicp.getFitnessScore();
  if(m_nano_gicp.hasConverged() && score < m_icp_score_thr) // if matchness score is lower than threshold, (lower is better)
  {
    if_converged = true;
    output_tf_ = m_nano_gicp.getFinalTransformation().cast<double>();
  }
  return output_tf_;  
}

Eigen::Matrix4d FastLioSamQnClass::coarseToFineSubkeysToSubkeys(const std::deque<PosePcd> &front_keyframes, const int& not_processed_idx,
                                                                const int &closest_idx, const std::vector<PosePcd> &keyframes, bool &if_converged, double &score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  // Prepare the keyframes
  pcl::PointCloud<PointType> dst_raw_, src_raw_, aligned_, src_coarse_aligned_, fine_aligned_;
  for (int i = not_processed_idx-m_sub_key_num; i < not_processed_idx+m_sub_key_num+1; ++i)
  {
    if (i>=0 && i < front_keyframes.size()-1) //if exists
    {
      src_raw_ += transformPcd(front_keyframes[i].pcd, front_keyframes[i].pose_corrected_eig);
    }
  }
  for (int i = closest_idx-m_sub_key_num; i < closest_idx+m_sub_key_num+1; ++i)
  {
    if (i>=0 && i < keyframes.size()-1) //if exists
    {
      dst_raw_ += transformPcd(keyframes[i].pcd, keyframes[i].pose_corrected_eig);
    }
  }
  // voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  // then perform Quatro
  Eigen::Matrix4d quatro_tf_ = m_quatro_handler->align(src_raw_, dst_raw_, if_converged);
  if (!if_converged) return quatro_tf_;
  else //if valid,
  {
    // coarse align with the result of Quatro
    src_coarse_aligned_ = transformPcd(src_raw_, quatro_tf_);
    // then match with Nano-GICP
    pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
    *dst_ = dst_raw_;
    *src_ = src_coarse_aligned_;
    m_nano_gicp.setInputSource(src_);
    m_nano_gicp.calculateSourceCovariances();
    m_nano_gicp.setInputTarget(dst_);
    m_nano_gicp.calculateTargetCovariances();
    m_nano_gicp.align(fine_aligned_);
    // handle results
    score = m_nano_gicp.getFitnessScore();
    if(m_nano_gicp.hasConverged() && score < m_icp_score_thr) // if matchness score is lower than threshold, (lower is better)
    {
      if_converged = true;
      Eigen::Matrix4d icp_tf_ = m_nano_gicp.getFinalTransformation().cast<double>();
      output_tf_ = icp_tf_ * quatro_tf_; // IMPORTANT: take care of the order
    }
    else if_converged = false;
    // vis for debug
    m_debug_src_pub.publish(pclToPclRos(src_raw_, m_map_frame));
    m_debug_dst_pub.publish(pclToPclRos(dst_raw_, m_map_frame));
    m_debug_coarse_aligned_pub.publish(pclToPclRos(src_coarse_aligned_, m_map_frame));
    m_debug_fine_aligned_pub.publish(pclToPclRos(fine_aligned_, m_map_frame));
  }

  return output_tf_;
}