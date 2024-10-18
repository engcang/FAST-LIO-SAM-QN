#include "fast_lio_sam_qn.h"


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

int FastLioSamQn::getClosestKeyframeIdx(const PosePcd &front_keyframe, const std::vector<PosePcd> &keyframes)
{
  const auto &m_loop_det_radi = lc_config_.loop_detection_radius_;
  const auto &m_loop_det_tdiff_thr = lc_config_.loop_detection_timediff_threshold_;
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
