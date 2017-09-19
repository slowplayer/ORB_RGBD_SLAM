#ifndef TOOLS_H
#define TOOLS_H

#include "g2o/types/slam3d/vertex_se3.h"
#include <Eigen/Core>
namespace ORB_RGBD_SLAM
{
g2o::SE3Quat eigen2G2O(const Eigen::Matrix4d& eigen_mat);
bool isSmallTrafo(const g2o::SE3Quat& t, double seconds);
bool isBigTrafo(const g2o::SE3Quat& t);
}
#endif