#ifndef FAST_LIO_SAM_QN_LOOP_CLOSURE_H
#define FAST_LIO_SAM_QN_LOOP_CLOSURE_H

///// Nano-GICP
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>
///// Quatro
#include <quatro/quatro_module.h>
///// coded headers
#include "utilities.hpp"

struct NanoGICPConfig {
  int    nano_thread_number_             = 0;
  int    nano_correspondences_number_    = 15;
  int    nano_max_iter_                  = 32;
  int    nano_ransac_max_iter_           = 5;

  double max_corr_dist_                      = 2.0;
  double icp_score_thr_                      = 10.0;
  double transformation_epsilon_             = 0.01;
  double euclidean_fitness_epsilon_          = 0.01;
  double ransac_outlier_rejection_threshold_ = 1.0;
};

struct QuatroConfig {
  bool   use_optimized_matching_         = true;
  bool   estimat_scale_                  = false;

  int    quatro_max_num_corres_          = 500;
  int    quatro_max_iter_                = 50;
  
  double quatro_distance_threshold_      = 30.0;
  double fpfh_normal_radius_             = 0.20; // It should be 2.5 - 3.0 * `voxel_res`
  double fpfh_radius_                    = 0.50; // It should be 5.0 * `voxel_res`
  double noise_bound_                    = 0.30;
  double rot_gnc_factor_                 = 1.40;
  double rot_cost_diff_thr_              = 0.0001;};


struct LoopClosureConfig {
  bool enable_quatro_          = true;
  bool enable_submap_matching_ = true;

  double voxel_res_ = 1.0;
  double loop_detection_radius_;
  double loop_detection_timediff_threshold_;
  
  NanoGICPConfig gicp_config_; 
  QuatroConfig quatro_config_; 
};

class LoopClosure
{
  private:
  
    nano_gicp::NanoGICP<PointType, PointType> m_nano_gicp;
    shared_ptr<quatro<PointType>> m_quatro_handler = nullptr;

    pcl::PointCloud<PointType> coarse_aligned_;
    pcl::PointCloud<PointType> aligned_;
    
    LoopClosureConfig config_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LoopClosure(const LoopClosureConfig &config);
    ~LoopClosure();

    int fetchClosestKeyframeIdx(const PosePcd &front_keyframe, const std::vector<PosePcd> &keyframes);

    std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>> setSrcAndDstCloud(std::vector<PosePcd> keyframes, const int src_idx, const int dst_idx, const int submap_range, const double voxel_res, const bool enable_quatro, const bool enable_submap_matching);
    RegistrationOutput icpAlignment(const pcl::PointCloud<PointType> &src_raw_, const pcl::PointCloud<PointType> &dst_raw_);
    RegistrationOutput coarseToFineAlignment(const pcl::PointCloud<PointType> &src_raw_, const pcl::PointCloud<PointType> &dst_raw_);
    RegistrationOutput retrieveLoopClosureMeasurement(const PosePcd &query_keyframe, const std::vector<PosePcd> &keyframes);

    pcl::PointCloud<PointType> getCoarseAlignedCloud();
    pcl::PointCloud<PointType> getFinalAlignedCloud();
};

#endif
