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
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>


//////////////////////////////////////////////////////////////////////
///// conversions
gtsam::Pose3 poseEigToGtsamPose(const Eigen::Matrix4d &pose_eig_in)
{
	double r_, p_, y_;
	tf::Matrix3x3 mat_;
	tf::matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat_);
	mat_.getRPY(r_, p_, y_);
	return gtsam::Pose3(gtsam::Rot3::RzRyRx(r_, p_, y_), gtsam::Point3(pose_eig_in(0, 3), pose_eig_in(1, 3), pose_eig_in(2, 3)));
}
Eigen::Matrix4d gtsamPoseToPoseEig(const gtsam::Pose3 &gtsam_pose_in)
{
	Eigen::Matrix4d pose_eig_out_ = Eigen::Matrix4d::Identity();
	tf::Quaternion quat_ = tf::createQuaternionFromRPY(gtsam_pose_in.rotation().roll(), gtsam_pose_in.rotation().pitch(), gtsam_pose_in.rotation().yaw());
	tf::Matrix3x3 mat_(quat_);
	Eigen::Matrix3d tmp_rot_mat_;
	tf::matrixTFToEigen(mat_, tmp_rot_mat_);
	pose_eig_out_.block<3, 3>(0, 0) = tmp_rot_mat_;
	pose_eig_out_(0, 3) = gtsam_pose_in.translation().x();
	pose_eig_out_(1, 3) = gtsam_pose_in.translation().y();
	pose_eig_out_(2, 3) = gtsam_pose_in.translation().z();
	return pose_eig_out_;
}
geometry_msgs::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in, std::string frame_id="map")
{
	double r_, p_, y_;
	tf::Matrix3x3 mat_;
	tf::matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat_);
	mat_.getRPY(r_, p_, y_);
	tf::Quaternion quat_ = tf::createQuaternionFromRPY(r_, p_, y_);
	geometry_msgs::PoseStamped pose_;
	pose_.header.frame_id = frame_id;
	pose_.pose.position.x = pose_eig_in(0, 3);
	pose_.pose.position.y = pose_eig_in(1, 3);
	pose_.pose.position.z = pose_eig_in(2, 3);
	pose_.pose.orientation.w = quat_.getW();
	pose_.pose.orientation.x = quat_.getX();
	pose_.pose.orientation.y = quat_.getY();
	pose_.pose.orientation.z = quat_.getZ();
	return pose_;
}
geometry_msgs::PoseStamped gtsamPoseToPoseStamped(const gtsam::Pose3 &gtsam_pose_in, std::string frame_id="map")
{
	tf::Quaternion quat_ = tf::createQuaternionFromRPY(gtsam_pose_in.rotation().roll(), gtsam_pose_in.rotation().pitch(), gtsam_pose_in.rotation().yaw());
	geometry_msgs::PoseStamped pose_;
	pose_.header.frame_id = frame_id;
	pose_.pose.position.x = gtsam_pose_in.translation().x();
	pose_.pose.position.y = gtsam_pose_in.translation().y();
	pose_.pose.position.z = gtsam_pose_in.translation().z();
	pose_.pose.orientation.w = quat_.getW();
	pose_.pose.orientation.x = quat_.getX();
	pose_.pose.orientation.y = quat_.getY();
	pose_.pose.orientation.z = quat_.getZ();
	return pose_;
}
template <typename T>
sensor_msgs::PointCloud2 pclToPclRos(pcl::PointCloud<T> cloud, std::string frame_id="map")
{
	sensor_msgs::PointCloud2 cloud_ROS_;
	pcl::toROSMsg(cloud, cloud_ROS_);
	cloud_ROS_.header.frame_id = frame_id;
	return cloud_ROS_;
}
///// transformation
template <typename T>
pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in, const Eigen::Matrix4d &pose_tf)
{
	if (cloud_in.size() == 0) return cloud_in;
	pcl::PointCloud<T> pcl_out_ = cloud_in;
	pcl::transformPointCloud(cloud_in, pcl_out_, pose_tf);
	return pcl_out_;
}



#endif