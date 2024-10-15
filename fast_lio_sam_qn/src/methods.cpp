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

std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>> FastLioSamQnClass::setSrcAndDstCloud(std::vector<PosePcd> keyframes, const int src_idx, const int dst_idx, const int submap_range, const bool enable_quatro, const bool enable_submap_matching)
{
  pcl::PointCloud<PointType> dst_raw_, src_raw_;
  int num_approx = keyframes[i].pcd.size() * 2 * submap_range;
  src_raw_.reserve(num_approx);
  dst_raw_.reserve(num_approx);
  if (enable_submap_matching)
  {
    for (int i = src_idx - submap_range; i < src_idx + submap_range + 1; ++i)
    {
      if (i >= 0 && i < keyframes.size()-1) // if exists
      {
        src_raw_ += transformPcd(keyframes[i].pcd, keyframes[i].pose_corrected_eig);
      }
    }
    for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i)
    {
      if (i >= 0 && i < keyframes.size()-1) //if exists
      {
        dst_raw_ += transformPcd(keyframes[i].pcd, keyframes[i].pose_corrected_eig);
      }
    }
  }
  else 
  {
    src_raw_ = transformPcd(keyframes[src_idx].pcd, keyframes[src_idx].pose_corrected_eig);
    if (enable_quatro) {
      dst_raw_ = transformPcd(keyframes[dst_idx].pcd, keyframes[dst_idx].pose_corrected_eig);
    } 
    else
    {
      // For ICP matching, 
      // empirically scan-to-submap matching works better
      for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i)
      {
        if (i >= 0 && i < keyframes.size()-1) //if exists
        {
          dst_raw_ += transformPcd(keyframes[i].pcd, keyframes[i].pose_corrected_eig);
        }
      }
    }
  }
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);

  return {src_raw_, dst_raw_};
}

Eigen::Matrix4d FastLioSamQnClass::icpAlignment(const pcl::PointCloud<PointType> &src_raw_, const pcl::PointCloud<PointType> &dst_raw_, bool &if_converged, double &score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType> aligned_;
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

Eigen::Matrix4d FastLioSamQnClass::coarseToFineAlignment(const pcl::PointCloud<PointType> &src_raw_, const pcl::PointCloud<PointType> &dst_raw_, bool &if_converged, double &score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d quatro_tf_ = m_quatro_handler->align(src_raw_, dst_raw_, if_converged);
  if (!if_converged) return quatro_tf_;
  else //if valid,
  {
    // coarse align with the result of Quatro
    const pcl::PointCloud<PointType> &src_coarse_aligned_ = transformPcd(src_raw_, quatro_tf_);
    // then match with Nano-GICP
    pcl::PointCloud<PointType> fine_aligned_;
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
