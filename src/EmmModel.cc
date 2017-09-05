#include "EmmModel.h"

namespace ORB_RGBD_SLAM
{
void observationLikelihood(const Eigen::Matrix4f& proposed_transformation,//new to old
                             pointcloud_type::Ptr new_pc,
                             pointcloud_type::Ptr old_pc,
                             const sensor_msgs::CameraInfo& old_cam_info,
                             double& likelihood, 
                             double& confidence,
                             unsigned int& inliers,
                             unsigned int& outliers,
                             unsigned int& occluded,
                             unsigned int& all) 
{
  ScopedTimer s(__FUNCTION__);
 
  int skip_step = ParameterServer::instance()->get<int>("emm__skip_step");
  const bool mark_outliers = ParameterServer::instance()->get<bool>("emm__mark_outliers");
  double observability_threshold = ParameterServer::instance()->get<double>("observability_threshold");
  inliers = outliers = occluded = all = 0;
  if(skip_step < 0 || observability_threshold <= 0.0){
    inliers = all = 1;
    return;
  }
  if(old_pc->width <= 1 || old_pc->height <= 1){
    //We need structured point clouds
    ROS_ERROR("Point Cloud seems to be non-structured: %u/%u (w/h). Observation can not be evaluated! ", old_pc->width, old_pc->height);
    if(ParameterServer::instance()->get<double>("voxelfilter_size") > 0){
      ROS_ERROR("The parameter voxelfilter_size is set. This is incompatible with the environment measurement model (parameter 'observability_threshold')");
    }
    inliers = all = 1;
    return;
  }
  if(old_pc->width != new_pc->width){
    ROS_ERROR("Differing cloud dimensions: %d vs %d width. Skipping observationLikelihood.", old_pc->width, new_pc->width);
    return;
  }

  pointcloud_type new_pc_transformed;
  pcl::transformPointCloud(*new_pc, new_pc_transformed, proposed_transformation);

  //Camera Calibration FIXME: Get actual values from cameraInfo (need to store in node?)
  ParameterServer* ps = ParameterServer::instance();
  float fx, fy, cx, cy;
  getCameraIntrinsics(fx, fy, cx, cy, old_cam_info);
  int cloud_creation_skip_step = 1; 
  if(ps->get<std::string>("topic_points").empty()){//downsampled cloud?
    cloud_creation_skip_step = ps->get<int>("cloud_creation_skip_step");
    fx = fx / cloud_creation_skip_step;
    fy = fy / cloud_creation_skip_step;
    cx = cx / cloud_creation_skip_step;
    cy = cy / cloud_creation_skip_step;
  }

  double sumloglikelihood = 0.0, observation_count = 0.0;
  unsigned int bad_points = 0, good_points = 0, occluded_points = 0;
  uint8_t r1 = rand() % 32, g1 = 128 + rand() % 128, b1 = 128+rand() % 128; // Mark occluded point in cyan color
  uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
  uint8_t r2 = 128 + rand() % 128, g2 = rand() % 32,  b2 = 128+rand() % 128; // Mark bad points in magenta color
  uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);

//#pragma omp parallel for reduction(+: good_points, bad_points, occluded_points) 
  for(int new_ry = 0; new_ry < (int)new_pc->height; new_ry+=skip_step){
    for(int new_rx = 0; new_rx < (int)new_pc->width; new_rx+=skip_step, all++){
      //Backproject transformed new 3D point to 2d raster of old image
      //TODO: Cache this?
      point_type& p = new_pc_transformed.at(new_rx, new_ry);
      if(p.z != p.z) continue; //NaN
      if(p.z < 0) continue; // Behind the camera
      int old_rx_center = round((p.x / p.z)* fx + cx);
      int old_ry_center = round((p.y / p.z)* fy + cy);
      //ROS_INFO_COND(new_ry % 32 == 0 && new_rx % 32 == 0, "Projected point from [%d, %d] to [%d, %d]", new_rx, new_ry, old_rx_center, old_ry_center);
      if(old_rx_center >= (int)old_pc->width || old_rx_center < 0 ||
         old_ry_center >= (int)old_pc->height|| old_ry_center < 0 )
      {
        ROS_DEBUG("New point not projected into old image, skipping");
        continue;
      }
      int nbhd = 2; //1 => 3x3 neighbourhood
      bool good_point = false, occluded_point = false, bad_point = false;
      int startx = std::max(0,old_rx_center - nbhd);
      int starty = std::max(0,old_ry_center - nbhd);
      int endx = std::min(static_cast<int>(old_pc->width), old_rx_center + nbhd +1);
      int endy = std::min(static_cast<int>(old_pc->height), old_ry_center + nbhd +1);
      int neighbourhood_step = 2; //Search for depth jumps in this area
      for(int old_ry = starty; old_ry < endy; old_ry+=neighbourhood_step){
        for(int old_rx = startx; old_rx < endx; old_rx+=neighbourhood_step){

          const point_type& old_p = old_pc->at(old_rx, old_ry);
          if(old_p.z != old_p.z) continue; //NaN
          
          // likelihood for old msrmnt = new msrmnt:
          double old_sigma = cloud_creation_skip_step*depth_covariance(old_p.z);
          //TODO: (Wrong) Assumption: Transformation does not change the viewing angle. 
          double new_sigma = cloud_creation_skip_step*depth_covariance(p.z);
          //Assumption: independence of sensor noise lets us sum variances
          double joint_sigma = old_sigma + new_sigma;
          ///TODO: Compute correctly transformed new sigma in old_z direction
          
          //Cumulative of position: probability of being occluded
          double p_new_in_front = cdf(old_p.z, p.z, sqrt(joint_sigma));
          //ROS_INFO("Msrmnt. dz=%g MHD=%g sigma=%g obs_p=%g, behind_p=%g", dz, mahal_dist, sqrt(joint_sigma), observation_p, p_new_in_front);
          if( p_new_in_front < 0.001)
          { // it is in behind and outside the 99.8% interval
            occluded_point = true; //Outside, but occluded
          }
          else if( p_new_in_front < 0.999)
          { // it is inside the 99.8% interval (and not behind)
            good_point = true;
            //goto end_of_neighbourhood_loop; <-- Don't break, search for better Mahal distance
          }
          else {//It would have blocked the view from the old cam position
            bad_point = true;
            //Bad point?
          }
          //sumloglikelihood += observation_p;
          //observation_count += p_new_in_front; //Discount by probability of having been occluded
        }
      }//End neighbourhood loop
      end_of_neighbourhood_loop:
      if(good_point) {
        good_points++;
      } else if(occluded_point){
        occluded_points++;
        if(mark_outliers){
#ifndef RGB_IS_4TH_DIM
          new_pc->at(new_rx, new_ry).rgb = *reinterpret_cast<float*>(&rgb1);
          old_pc->at(old_rx_center, old_ry_center).rgb = *reinterpret_cast<float*>(&rgb1);
#else
          new_pc->at(new_rx, new_ry).data[3] = *reinterpret_cast<float*>(&rgb1);
          old_pc->at(old_rx_center, old_ry_center).data[3] = *reinterpret_cast<float*>(&rgb1);
#endif
        }
      }
      else if(bad_point){
        bad_points++;
        if(mark_outliers){
          //uint8_t r1 = 255, g1 = 0, b1 = 0; // Mark bad point in red color
#ifndef RGB_IS_4TH_DIM
          new_pc->at(new_rx, new_ry).rgb = *reinterpret_cast<float*>(&rgb2);
          old_pc->at(old_rx_center, old_ry_center).rgb = *reinterpret_cast<float*>(&rgb2);
#else
          new_pc->at(new_rx, new_ry).data[3] = *reinterpret_cast<float*>(&rgb2);
          old_pc->at(old_rx_center, old_ry_center).data[3] = *reinterpret_cast<float*>(&rgb2);
#endif
          //Kill point
          //new_pc->at(new_rx, new_ry).z = std::numeric_limits<float>::quiet_NaN();
          //old_pc->at(old_rx_center, old_ry_center).z = std::numeric_limits<float>::quiet_NaN();
        }
      }
      else {} //only NaN?
    }
  }
  likelihood = sumloglikelihood/observation_count;//more readable
  confidence = observation_count;
  inliers = good_points;
  outliers = bad_points;
  occluded = occluded_points;
}
void pairwiseObservationLikelihood(const Node* newer_node, const Node* older_node, MatchingResult& mr)
{ 
      double likelihood, confidence;
      unsigned int inlier_points = 0, outlier_points = 0, all_points = 0, occluded_points = 0;
          
      unsigned int inlier_pts = 0, outlier_pts = 0, occluded_pts = 0, all_pts = 0;
     
      observationLikelihood(mr.final_trafo, newer_node->pc_col, older_node->pc_col, older_node->getCamInfo(), likelihood, confidence, inlier_pts, outlier_pts, occluded_pts, all_pts) ;
      //rejectionSignificance(mr.final_trafo, newer_node->pc_col, older_node->pc_col);
      inlier_points += inlier_pts;
      outlier_points += outlier_pts;
      occluded_points += occluded_pts;
      all_points += all_pts;
    
      mr.inlier_points = inlier_points;
      mr.outlier_points = outlier_points;
      mr.occluded_points = occluded_points;
      mr.all_points = all_points;
}
bool observation_criterion_met(unsigned int inliers, unsigned int outliers, unsigned int all, double& quality)
{
  double obs_thresh1 = ParameterServer::instance()->getParam("observability_threshold1");
  double obs_thresh2 = ParameterServer::instance()->getParam("observability_threshold2");
  quality = inliers/static_cast<double>(inliers+outliers);
  double certainty = inliers/static_cast<double>(all);
  bool criterion1_met = quality > obs_thresh1; //TODO: parametrice certainty (and use meaningful statistic)
  bool criterion2_met = certainty > obs_thresh2; //TODO: parametrice certainty (and use meaningful statistic)
  bool both_criteria_met = criterion1_met && criterion2_met;
  return both_criteria_met;
}

}