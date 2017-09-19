#include "tools.h"
#include "ParameterServer.h"
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
bool isSmallTrafo(const g2o::SE3Quat& t, double seconds)
{
    if(seconds <= 0.0){
      return true;
    }
    float angle_around_axis = 2.0*acos(t.rotation().w()) *180.0 / M_PI;
    float dist = t.translation().norm();
    //Q_EMIT setGUIInfo2(infostring);
    ParameterServer* ps =  ParameterServer::instance();
    //Too big fails too
    return (dist / seconds < ps->getParam("max_translation_meter") &&
            angle_around_axis / seconds < ps->getParam("max_rotation_degree"));
}

bool isBigTrafo(const g2o::SE3Quat& t)
{
    float angle_around_axis = 2.0*acos(t.rotation().w()) *180.0 / M_PI;
    float dist = t.translation().norm();
    //Q_EMIT setGUIInfo2(infostring);
    ParameterServer* ps =  ParameterServer::instance();
    return (dist > ps->get<double>("min_translation_meter") ||
            angle_around_axis > ps->get<double>("min_rotation_degree"));
}
  
}