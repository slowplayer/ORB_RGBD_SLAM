#ifndef MISC_H
#define MISC_H

#include <Eigen/Core>
#include "g2o/types/slam3d/se3quat.h"
#include "ParameterServer.h"

namespace ORB_RGBD_SLAM
{
inline double depth_std_dev(double depth)
{
  // From Khoselham and Elberink?
  static double depth_std_dev = ParameterServer::instance()->getParam("sigma_depth");
  // Previously used 0.006 from information on http://www.ros.org/wiki/openni_kinect/kinect_accuracy;
  // ...using 2sigma = 95%ile
  //static const double depth_std_dev  = 0.006;
  return depth_std_dev * depth * depth;
}
//Functions without dependencies
inline double depth_covariance(double depth)
{
  static double stddev = depth_std_dev(depth);
  static double cov = stddev * stddev;
  return cov;
}
inline g2o::SE3Quat eigen2G2O(const Eigen::Matrix4d& eigen_mat) 
{
  Eigen::Affine3d eigen_transform(eigen_mat);
  Eigen::Quaterniond eigen_quat(eigen_transform.rotation());
  Eigen::Vector3d translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
  g2o::SE3Quat result(eigen_quat, translation);

  return result;
}
double errorFunction2(const Eigen::Vector4f& x1,const Eigen::Vector4f& x2,
		    const Eigen::Matrix4d& transformation);
}