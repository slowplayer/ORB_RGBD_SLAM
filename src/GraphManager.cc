#include "GraphManager.h"


namespace ORB_RGBD_SLAM
{

GraphManager::GraphManager()
:next_seq_id(0),next_vertex_id(0)
{
}
bool GraphManager::addNode(Node* new_node)
{
  if(new_node->feature_location_2d_.size()<min_matches)
  {
    return false;
  }
  if(graph_.size()==0)
  {
    firstNode(new_node);
    return true;
  }
  return otherNode(new_node);
}
void GraphManager::firstNode(Node* new_node)
{
  new_node->id_=graph_.size();
  new_node->seq_id_=next_seq_id++;
  new_node->vertex_id_=next_vertex_id++;
  graph_[new_node->id_]=new_node;
  
  //TODO:add the first vertex to g2o graph
  
  
  addKeyframe(new_node->id_);
  
  optimizeGraph();
}
bool GraphManager::otherNode(Node* new_node)
{
  Eigen::Matrix4f motion_estimate;
  bool edge_to_last_keyframe_found=false;
  bool found_match=nodeComparisons(new_node,curr_motion_estimate,edge_to_last_keyframe_found);
  
}
void GraphManager::addKeyframe(int id)
{
  keyframe_ids_.push_back(id);
}
bool GraphManager::nodeComparisons(Node* new_node, Eigen::Matrix4f& curr_motion_estimate, bool& edge_to_keyframe)
{
  
  //Initial Comparison
  MatchingResult mr;
  Node* prev_node=graph_[graph_.size()-1];
  mr=new_node->matchNodePair(prev_node);
}
void GraphManager::optimizeGraph()
{
  
}

}