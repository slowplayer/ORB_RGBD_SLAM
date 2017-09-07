#include "misc.h"

namespace ORB_RGBD_SLAM
{
typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;
pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_msg, const cv::Mat& rgb_msg)
{
  point_type* cloud(new point_type());
  cloud->dense=false;
  
  ParameterServer* ps=ParameterServer::instance();
  float fxinv,fyinv,cx,cy;
  fxinv=1.0/ps->getParam("fx");
  fyinv=1.0/ps->getParam("fy");
  cx=ps->getParam("cx");
  cy=ps->getParam("cy");
  int data_skip_step=static_cast<int>(ps->getParam("cloud_creation_skip_step"));
  
  cloud->height=ceil(depth_msg.rows/static_cast<float>(data_skip_step));
  cloud->width=ceil(depth_msg.cols/static_cast<float>(data_skip_step));
  
  char red_idx=0,green_idx=1,blue_idx=2; //RGB-format
  
  unsigned int depth_pix_step=(depth_msg.cols/cloud->width);
  unsigned int depth_row_step=(depth_msg.rows/cloud->height-1)*depth_msg.cols;
  unsigned int color_pix_step=3*depth_pix_step;
  unsigned int color_row_step=3*depth_row_step;
  
  cloud->resize(cloud->height*cloud->width);
  
  int color_idx=0;
  int depth_idx=0;
  double depth_scaling=static_cast<double>(ps->getParam("depth_scaling_factor"));
  float max_depth=ps->getParam("maximum_depth");
  float min_depth=ps->getParam("minimum_depth");
  if(max_depth<0.0)max_depth=std::numeric_limits<float>::infinity();
  
  pointcloud_type::iterator pt_iter;
  for(int v=0;v<(int)rgb_msg.rows;v+=data_skip_step,color_idx+=color_row_step;depth_idx+=depth_row_step)
  {
    for(int u=0;u<(int)rgb_msg.cols;u+=data_pix_step,color_idx+=color_pix_step;depth_idx+=depth_pix_step)
    {
      if(pt_iter==cloud->end())
	break;
      point_type& pt=*pt_iter;
      float Z=depth_msg.at<float>(depth_idx)*depth_scaling;
      if(!(Z>=min_depth))
      {
	pt.x=(u-cx)*1.0*fxinv;
	pt.y=(v-cy)*1.0*fyinv;
	pt.z=std::numeric_limits< float >::quiet_NaN();
      }
      else
      {
	 pt.x=(u-cx)*Z*fxinv;
	 pt.y=(v-cy)*Z*fyinv;
	 pt.z=Z;
      }
      RGBValue color;
      if(color_idx>0&&color_idx<rgb_msg.total()*color_pix_step)
      {
	color.Red=rgb_msg.at<uint8_t>(color_idx+red_idx);
	color.Green=rgb_msg.at<uint8_t>(color_idx+green_idx);
	color.Blue=rgb_msg.at<uint8_t>(color_idx+blue_idx);
	color.Alpha=0;
	pt.data[3]=color.float_value;
      }
    }
  }
  return cloud;
}
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