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
//TODO
std::list< int > GraphManager::getPotentialEdgeTargetsWithDijkstra(const Node* new_node, int seq_targets, int geod_targets, int samp_targets, int prodecessor_id, bool include_predecessor)
{
  std::list<int> ids_to_link_to;
  if(prodecessor_id<0)
    prodecessor_id=graph_.size()-1;
  
  //TODO::the number of vertices is not enough
  
  //Time continuous
  if(seq_targets>0)
  {
    for(int i=0;i<seq_targets+1&&prodecessor_id-i>=0;i++)
    {
      ids_to_link_to.push_back(prodecessor_id-i);
    }
  }
  //TODO
  //spatial continuous
  if(geod_targets>0)
  {
    
  }
  //Geodesic Neighbours except sequential
  
  //Sample targets from graph-neighbours
  
  
  
  return ids_to_link_to;
}

bool GraphManager::nodeComparisons(Node* new_node, Eigen::Matrix4f& curr_motion_estimate, bool& edge_to_keyframe)
{
  
  //Initial Comparison
  MatchingResult mr;
  Node* prev_node=graph_[graph_.size()-1];
  mr=new_node->matchNodePair(prev_node);
  
  //TODO:evaluate mr and add egde to g2o
  
  //Get Potenial Edge
  std::list<int> vertices_to_comp;
  //TODO:add prev_best or not
  vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, curr_best_result_.edge.id1); 
  
  //Compare node pairs
  std::list<const Node*> nodes_to_comp;
  for(std::list<int>::iterator it=vertices_to_comp.begin();it!=vertices_to_comp.end();it++)
  {
    nodes_to_comp.push_back(graph_[*it]);
  }
  
  //TODO:run in parallel
  std::list<MatchingResult> results;
  for(std::list<const Node*>::iterator it=nodes_to_comp.begin();it!=nodes_to_comp.end();it++)
  {
    results.push_back(new_node->matchNodePair(*it));
  }
  
  for(std::list<MatchingResult>::iterator it=results.begin();it!=results.end();it++)
  {
    MatchingResult mr=*it;
  }
  
}
void GraphManager::optimizeGraph()
{
  
}

}