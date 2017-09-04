#ifndef NODE_H
#define NODE_H

#include <set>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "features.h"
#include "MatchingResult.h"
#include "ParameterServer.h"
#include "IcpSolver.h"
#include "misc.h"

namespace ORB_RGBD_SLAM
{
typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;
class Node
{
public:
  Node(const cv::Mat& imGray,const cv::Mat& imDepth,
       const cv::Mat& detector_mask,double timestamp,
       cv::Ptr<cv::Feature2D> detector,
       cv::Ptr<cv::DescriptorExtractor> extractor);
  Node(){};
  ~Node();
   
  MatchingResult matchNodePair(const Node* older_node);

  
  //2d-features,
  std::vector<cv::KeyPoint> feature_location_2d_;
  //3d-coordinates
  std_vector_of_eigen_vector4f feature_location_3d_;
  //features-descriptor
  cv::Mat features_descriptors_;
  
  int id_,seq_id_,vertex_id_;
  double timestamp_;
  int init_node_matches_;
  
  bool matchable_;
private:
  void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);
  void removeDepthless(std::vector<cv::KeyPoint>& feature_location_2d,const cv::Mat& depth);
  void projectTo3D(std::vector<cv::KeyPoint>& feature_location_2d,
    std_vector_of_eigen_vector4f& feature_location_3d,const cv::Mat& depth);
  
  unsigned int featureMatching(const Node* other,std::vector<cv::DMatch>* matches)const;
  bool getRelativeTransformationTo(const Node* earlier_node,
				  std::vector<cv::DMatch>* initial_matches,
				   Eigen::Matrix4f& resulting_transformation,
				   float& rmse,std::vector<cv::DMatch>& matches)const;
  std::vector<cv::DMatch> sample_matches_prefer_by_distance(unsigned int sample_size,
					std::vector<cv::DMatch>& matches_with_depth);
  Eigen::Matrix4f getTransformFromMatches(const Node* newer_node,const Node* earlier_node,
					  const std::vector<cv::DMatch>& matches,
					  bool& valid,const float max_dist_m);
  
  void computeInliersAndError(const std::vector<cv::DMatch> & all_matches,
                                  const Eigen::Matrix4f& transformation4f,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  size_t min_inliers, //break if this can't be reached
                                  std::vector<cv::DMatch>& inliers, //pure output var
                                  double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                  //std::vector<double>& errors,
                                  double squaredMaxInlierDistInM) const;				   
  void processNode(Node* node_ptr);
  
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
#endif