#ifndef FAST_LIO_SAM_QN_UTILITY_H
#define FAST_LIO_SAM_QN_UTILITY_H

///// common headers
#include <string>
///// ROS
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler
#include <tf/transform_datatypes.h> // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h> // tf <-> eigen
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h> //voxelgrid
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

using PointType = pcl::PointXYZI;

// struct PosePcd
// {
//   pcl::PointCloud<PointType> pcd_;
//   Eigen::Matrix4d pose_eig_           = Eigen::Matrix4d::Identity();
//   Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
//   double timestamp_;
//   int idx_;
//   bool processed_ = false;
//   PosePcd(){};
//   PosePcd(const nav_msgs::Odometry &odom_in, const sensor_msgs::PointCloud2 &pcd_in, const int &idx_in);
// };

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType> &pcd_in, const float voxel_res)
{
  static pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
  pcl::PointCloud<PointType>::Ptr pcd_in_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
  pcd_in_ptr->reserve(pcd_in.size());
  pcd_out->reserve(pcd_in.size());
  *pcd_in_ptr = pcd_in;
  voxelgrid.setInputCloud(pcd_in_ptr);
  voxelgrid.filter(*pcd_out);
  return pcd_out;
}

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType>::Ptr &pcd_in, const float voxel_res)
{
  static pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
  pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
  pcd_out->reserve(pcd_in->size());
  voxelgrid.setInputCloud(pcd_in);
  voxelgrid.filter(*pcd_out);
  return pcd_out;
}

//////////////////////////////////////////////////////////////////////
///// conversions
inline gtsam::Pose3 poseEigToGtsamPose(const Eigen::Matrix4d &pose_eig_in)
{
	double r, p, y;
	tf::Matrix3x3 mat;
	tf::matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat);
	mat.getRPY(r, p, y);
	return gtsam::Pose3(gtsam::Rot3::RzRyRx(r, p, y), gtsam::Point3(pose_eig_in(0, 3), pose_eig_in(1, 3), pose_eig_in(2, 3)));
}
inline Eigen::Matrix4d gtsamPoseToPoseEig(const gtsam::Pose3 &gtsam_pose_in)
{
	Eigen::Matrix4d pose_eig_out = Eigen::Matrix4d::Identity();
	tf::Quaternion quat = tf::createQuaternionFromRPY(gtsam_pose_in.rotation().roll(), gtsam_pose_in.rotation().pitch(), gtsam_pose_in.rotation().yaw());
	tf::Matrix3x3 mat(quat);
	Eigen::Matrix3d tmp_rot_mat;
	tf::matrixTFToEigen(mat, tmp_rot_mat);
	pose_eig_out.block<3, 3>(0, 0) = tmp_rot_mat;
	pose_eig_out(0, 3) = gtsam_pose_in.translation().x();
	pose_eig_out(1, 3) = gtsam_pose_in.translation().y();
	pose_eig_out(2, 3) = gtsam_pose_in.translation().z();
	return pose_eig_out;
}
inline geometry_msgs::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in, std::string frame_id="map")
{
	double r, p, y;
	tf::Matrix3x3 mat;
	tf::matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat);
	mat.getRPY(r, p, y);
	tf::Quaternion quat = tf::createQuaternionFromRPY(r, p, y);
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = frame_id;
	pose.pose.position.x = pose_eig_in(0, 3);
	pose.pose.position.y = pose_eig_in(1, 3);
	pose.pose.position.z = pose_eig_in(2, 3);
	pose.pose.orientation.w = quat.getW();
	pose.pose.orientation.x = quat.getX();
	pose.pose.orientation.y = quat.getY();
	pose.pose.orientation.z = quat.getZ();
	return pose;
}
inline geometry_msgs::PoseStamped gtsamPoseToPoseStamped(const gtsam::Pose3 &gtsam_pose_in, std::string frame_id="map")
{
	tf::Quaternion quat = tf::createQuaternionFromRPY(gtsam_pose_in.rotation().roll(), gtsam_pose_in.rotation().pitch(), gtsam_pose_in.rotation().yaw());
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = frame_id;
	pose.pose.position.x = gtsam_pose_in.translation().x();
	pose.pose.position.y = gtsam_pose_in.translation().y();
	pose.pose.position.z = gtsam_pose_in.translation().z();
	pose.pose.orientation.w = quat.getW();
	pose.pose.orientation.x = quat.getX();
	pose.pose.orientation.y = quat.getY();
	pose.pose.orientation.z = quat.getZ();
	return pose;
}
template <typename T>
inline sensor_msgs::PointCloud2 pclToPclRos(pcl::PointCloud<T> cloud, std::string frame_id="map")
{
	sensor_msgs::PointCloud2 cloud_ROS;
	pcl::toROSMsg(cloud, cloud_ROS);
	cloud_ROS.header.frame_id = frame_id;
	return cloud_ROS;
}
///// transformation
template <typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in, const Eigen::Matrix4d &pose_tf)
{
	if (cloud_in.size() == 0) return cloud_in;
	pcl::PointCloud<T> pcl_out = cloud_in;
	pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
	return pcl_out;
}

#endif
