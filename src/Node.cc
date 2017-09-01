#include "Node.h"

namespace ORB_RGBD_SLAM
{
 Node(const cv::Mat& imGray,const cv::Mat& imDepth,
       const cv::Mat& detector_mask,double timestamp,
       cv::Ptr<cv::Feature2D> detector,
       cv::Ptr<cv::DescriptorExtractor> extractor)
:id_(-1),seq_id_(-1),vertex_id_(-1),init_node_matches_(0),timestamp_(timestamp)
{
  detector->detect(imGray,feature_location_2d_,detector_mask);
  
  extractor->compute(imGray,feature_location_2d_,features_descriptors_);
  
  projectTo3D(feature_location_2d_,feature_location_3d_,imDepth);
  
  
}
void Node::projectTo3D(std::vector< cv::KeyPoint >& feature_location_2d, std_vector_of_eigen_vector4f feature_location_3d, const cv::Mat& depth)
{
  ParameterServer* ps=ParameterServer::instance();
}

MatchingResult Node::matchNodePair(const Node* older_node)
{

}

  
}