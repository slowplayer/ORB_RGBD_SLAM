#include "tools.h"

namespace ORB_RGBD_SLAM
{
g2o::SE3Quat eigen2G2O(const Eigen::Matrix4d& eigen_mat) 
{
  Eigen::Affine3d eigen_transform(eigen_mat);
  Eigen::Quaterniond eigen_quat(eigen_transform.rotation());
  Eigen::Vector3d translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
  g2o::SE3Quat result(eigen_quat, translation);

  return result;
} 
  
  
}