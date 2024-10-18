#include "loop_closure.h"

LoopClosure::LoopClosure(const LoopClosureConfig &config) 
{
  config_ = config;

  const auto & gc = config_.gicp_config_;
  const auto & qc = config_.quatro_config_;
  ////// nano_gicp init
  m_nano_gicp.setNumThreads(gc.nano_thread_number_);
  m_nano_gicp.setCorrespondenceRandomness(gc.nano_correspondences_number_);
  m_nano_gicp.setMaximumIterations(gc.nano_max_iter_);
  m_nano_gicp.setRANSACIterations(gc.nano_ransac_max_iter_);
  m_nano_gicp.setMaxCorrespondenceDistance(gc.max_corr_dist_);
  m_nano_gicp.setTransformationEpsilon(gc.transformation_epsilon_);
  m_nano_gicp.setEuclideanFitnessEpsilon(gc.euclidean_fitness_epsilon_);
  m_nano_gicp.setRANSACOutlierRejectionThreshold(gc.ransac_outlier_rejection_threshold_);
  ////// quatro init
  m_quatro_handler = std::make_shared<quatro<PointType>>(qc.fpfh_normal_radius_, qc.fpfh_radius_, qc.noise_bound_, qc.rot_gnc_factor_, qc.rot_cost_diff_thr_,
                                                        qc.quatro_max_iter_, qc.estimat_scale_, qc.use_optimized_matching_, qc.quatro_distance_threshold_, qc.quatro_max_num_corres_);
}

LoopClosure::~LoopClosure() {}

int LoopClosure::fetchClosestKeyframeIdx(const PosePcd &front_keyframe, const std::vector<PosePcd> &keyframes)
{
  const auto &loop_det_radi = config_.loop_detection_radius_;
  const auto &loop_det_tdiff_thr = config_.loop_detection_timediff_threshold_;

  double shortest_distance_ = loop_det_radi * 3.0;
  int closest_idx_ = -1;
  for (int idx = 0; idx < keyframes.size()-1; ++idx)
  {
    //check if potential loop: close enough in distance, far enough in time
    double tmp_dist = (keyframes[idx].pose_corrected_eig.block<3, 1>(0, 3) - front_keyframe.pose_corrected_eig.block<3, 1>(0, 3)).norm();
    if (loop_det_radi > tmp_dist && loop_det_tdiff_thr < (front_keyframe.timestamp - keyframes[idx].timestamp))
    {
      if (tmp_dist < shortest_distance_)
      {
        shortest_distance_ = tmp_dist;
        closest_idx_ = keyframes[idx].idx;
      }
    }
  }
  return closest_idx_;
}

std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>> LoopClosure::setSrcAndDstCloud(std::vector<PosePcd> keyframes, const int src_idx, const int dst_idx, const int submap_range, const double voxel_res, const bool enable_quatro, const bool enable_submap_matching)
{
  pcl::PointCloud<PointType> dst_raw_, src_raw_;
  int num_approx = keyframes[src_idx].pcd.size() * 2 * submap_range;
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
  return {*voxelizePcd(src_raw_, voxel_res), *voxelizePcd(dst_raw_, voxel_res)};
}

RegistrationOutput LoopClosure::icpAlignment(const pcl::PointCloud<PointType> &src_raw_, const pcl::PointCloud<PointType> &dst_raw_)
{
  RegistrationOutput reg_output;
  aligned_.clear(); 
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
  *dst_ = dst_raw_;
  *src_ = src_raw_;
  m_nano_gicp.setInputSource(src_);
  m_nano_gicp.calculateSourceCovariances();
  m_nano_gicp.setInputTarget(dst_);
  m_nano_gicp.calculateTargetCovariances();
  m_nano_gicp.align(aligned_);
  
  // handle results
  reg_output.score = m_nano_gicp.getFitnessScore();
  if(m_nano_gicp.hasConverged() && reg_output.score < config_.gicp_config_.icp_score_thr_) // if matchness score is lower than threshold, (lower is better)
  {
    reg_output.is_converged = true;
    reg_output.pose_between_eig = m_nano_gicp.getFinalTransformation().cast<double>();
  }
  return reg_output;
}

RegistrationOutput LoopClosure::coarseToFineAlignment(const pcl::PointCloud<PointType> &src_raw_, const pcl::PointCloud<PointType> &dst_raw_)
{
  RegistrationOutput reg_output;
  coarse_aligned_.clear(); 
  reg_output.pose_between_eig = (m_quatro_handler->align(src_raw_, dst_raw_, reg_output.is_converged));
  if (!reg_output.is_converged) return reg_output;
  else //if valid,
  {
    // coarse align with the result of Quatro
    coarse_aligned_ = transformPcd(src_raw_, reg_output.pose_between_eig);
    const auto &fine_output = icpAlignment(coarse_aligned_, dst_raw_);

    const auto quatro_tf_       = reg_output.pose_between_eig;
    reg_output.pose_between_eig = fine_output.pose_between_eig * quatro_tf_; // IMPORTANT: take care of the order
    reg_output.is_converged     = fine_output.is_converged;
    reg_output.score            = fine_output.score;
  }
  return reg_output;
}

RegistrationOutput LoopClosure::retrieveLoopClosureMeasurement(const PosePcd &query_keyframe, const std::vector<PosePcd> &keyframes) 
{
  RegistrationOutput reg_output;
  closest_keyframe_idx_ = fetchClosestKeyframeIdx(query_keyframe, keyframes);
  if (closest_keyframe_idx_ >= 0) 
  {
    // Quatro + NANO-GICP to check loop (from front_keyframe to closest keyframe's neighbor)
    const auto &[src_cloud, dst_cloud] = setSrcAndDstCloud(keyframes, query_keyframe.idx, closest_keyframe_idx_,
                                                                      config_.num_submap_keyframes_, config_.voxel_res_,
                                                                      config_.enable_quatro_, config_.enable_submap_matching_);
    // Only for visualization
    *src_cloud_ = src_cloud;
    *dst_cloud_ = dst_cloud;

    if (config_.enable_quatro_) 
    {
      std::cout << "\033[1;35mExecute coarse-to-fine alignment: "  << src_cloud.size() << " vs " << dst_cloud.size() << "\033[0m\n";
      return coarseToFineAlignment(src_cloud, dst_cloud);
    }
    else
    {
      std::cout << "\033[1;35mExecute GICP: "  << src_cloud.size() << " vs " << dst_cloud.size() << "\033[0m\n";
      return icpAlignment(src_cloud, dst_cloud);
    }
  } 
  else
  {
    return reg_output; // dummy output whose `is_valid` is false
  }
}

pcl::PointCloud<PointType> LoopClosure::getSourceCloud()
{
  return *src_cloud_;
}

pcl::PointCloud<PointType> LoopClosure::getTargetCloud()
{
  return *dst_cloud_;
}

pcl::PointCloud<PointType> LoopClosure::getCoarseAlignedCloud()
{
  return coarse_aligned_;
}

// NOTE(hlim): To cover ICP-only mode, I just set `Final`, not `Fine`
pcl::PointCloud<PointType> LoopClosure::getFinalAlignedCloud()
{
  return aligned_;
}

int LoopClosure::getClosestKeyframeidx()
{
  return closest_keyframe_idx_;
}

