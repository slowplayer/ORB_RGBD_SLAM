#include "misc.h"

namespace ORB_RGBD_SLAM
{
double errorFunction2(const Eigen::Vector4f& x1, const Eigen::Vector4f& x2, const Eigen::Matrix4d& transformation)
{
  
  ParameterServer* ps=ParameterServer::instance();
  static const double cam_angle_x=ps->getParam("cam_angle_x");
  static const double cam_angle_y=ps->getParam("cam_angle_y");
  static const double cam_resol_x=ps->getParam("cam_resol_x");
  static const double cam_resol_y=ps->getParam("cam_resol_y");
  
  static const double raster_stddev_x=3*tan(cam_angle_x/cam_resol_x);
  static const double raster_stddev_y=3*tan(cam_angle_y/cam_resol_y);
  
  static const double raster_cov_x=raster_stddev_x*raster_stddev_x;
  static const double raster_cov_y=raster_stddev_y*raster_stddev_y;
  
  
  if(std::isnan(x1(2))||std::isnan(x2(2)))
    return std::numeric_limits< double >::max();
  
  Eigen::Vector4d x_1=x1.cast<double>();
  Eigen::Vector4d x_2=x2.cast<double>();
  
  Eigen::Matrix4d tf_12=transformation;
  Eigen::Vector3d mu_1=x_1.head<3>();
  Eigen::Vector3d mu_2=x_2.head<3>();
  Eigen::Vector3d mu_1_in_frame2=(tf_12*x_1).head<3>();
  
  
  //Euclidean distance
  double delta_sq_norm=(mu_1_in_frame2- mu_2).squaredNorm();
  double sigma_max_1=std::max(raster_cov_x,depth_covariance(mu_1(2)));
  double sigma_max_2=std::max(raster_cov_x,depth_covariance(mu_2(2)));
  if(delta_sq_norm>2.0*(sigma_max_1+sigma_max_2))
  {
     return std::numeric_limits< double >::max();
  }
  
  //Mahalanobis distance
  Eigen::Matrix3d rotation_mat=tf_12.block(0,0,3,3);
  
  //Point 1
  Eigen::Matrix3d cov1=Eigen::Matrix3d::Zero();
  cov1(0,0)=raster_cov_x*mu_1(2);
  cov1(1,1)=raster_cov_y*mu_1(2);
  cov1(2,2)=depth_covariance(mu_1(2));
  //Point 2
  Eigen::Matrix3d cov2=Eigen::Matrix3d::Zero();
  cov2(0,0)=raster_cov_x*mu_2(2);
  cov2(1,1)=raster_cov_y*mu_2(2);
  cov2(2,2)=depth_covariance(mu_2(2));
  
  Eigen::Matrix3d cov1_in_frame_2=rotation_mat.transpose()*cov1*rotation_mat;
  delta_mu_in_frame2=mu_1_in_frame2-mu_2;
  if(std::isnan(delta_mu_in_frame2(2)))
  {
      return std::numeric_limits< double >::max();
  }
  Eigen::Matrix3d cov_mat_sum_in_frame_2=cov1_in_frame_2+cov2;
  double sqrd_mahalanobis_distance=delta_sq_norm.transpose()*cov_mat_sum_in_frame_2.llt().solve(delta_mu_in_frame2);
  
  if(!(sqrd_mahalanobis_distance>=0.0))
  {
    return std::numeric_limits<double>::max();
  }
  return sqrd_mahalanobis_distance;
}

}