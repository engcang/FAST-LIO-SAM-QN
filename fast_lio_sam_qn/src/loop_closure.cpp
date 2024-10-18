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

  ROS_INFO("Loop Closure Started...");
}

LoopClosure::~LoopClosure() {}

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
  // m_debug_src_pub.publish(pclToPclRos(src_raw_, m_map_frame));
  // m_debug_dst_pub.publish(pclToPclRos(dst_raw_, m_map_frame));
  // m_debug_fine_aligned_pub.publish(pclToPclRos(aligned_, m_map_frame));
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
  reg_output.pose_between_eig = (m_quatro_handler->align(src_raw_, dst_raw_, reg_output.is_converged));
  if (!reg_output.is_converged) return reg_output;
  else //if valid,
  {
    // coarse align with the result of Quatro
    const pcl::PointCloud<PointType> &src_coarse_aligned_ = transformPcd(src_raw_, reg_output.pose_between_eig);
    const auto &fine_output = icpAlignment(src_coarse_aligned_, dst_raw_);

    const auto quatro_tf_       = reg_output.pose_between_eig;
    reg_output.pose_between_eig = fine_output.pose_between_eig * quatro_tf_; // IMPORTANT: take care of the order
    reg_output.is_converged     = fine_output.is_converged;
    reg_output.score            = fine_output.score;

    // m_debug_coarse_aligned_pub.publish(pclToPclRos(src_coarse_aligned_, m_map_frame));
  }
  return reg_output;
}
