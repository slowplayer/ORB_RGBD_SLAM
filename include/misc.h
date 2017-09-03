#ifndef MISC_H
#define MISC_H

#include <Eigen/Core>

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
double errorFunction2(const Eigen::Vector4f& x1,const Eigen::Vector4f& x2,
		    const Eigen::Matrix4d& transformation);
}