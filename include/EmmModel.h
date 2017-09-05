#ifndef EMM_MODEL_H
#define EMM_MODEL_H

#include "MatchingResult.h"
#include "Node.h"
#include "ParameterServer.h"

namespace ORB_RGBD_SLAM
{
inline double cdf(double x, double mu, double sigma)
{
	return 0.5 * (1 + erf((x - mu) / (sigma * 1.41421)));
}
void observationLikelihood(const Eigen::Matrix4f& proposed_transformation,//new to old
                             pointcloud_type::Ptr new_pc,
                             pointcloud_type::Ptr old_pc,
                             const sensor_msgs::CameraInfo& old_cam_info,
                             double& likelihood, 
                             double& confidence,
                             unsigned int& inliers,
                             unsigned int& outliers,
                             unsigned int& occluded,
                             unsigned int& all);
void pairwiseObservationLikelihood(const Node* newer_node, const Node* older_node, MatchingResult& mr);
bool observation_criterion_met(unsigned int inliers, unsigned int outliers, unsigned int all, double& quality);
}
#endif