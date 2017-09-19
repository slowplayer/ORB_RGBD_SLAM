#include "Tracking.h"

namespace ORB_RGBD_SLAM
{
Tracking::Tracking()
:eState(SYSTEM_NOT_READY),
next_seq_id(0),next_vertex_id(0)
{

}
void Tracking::setOptimizer(Optimizer* pOptimizer)
{
  mpOptimizer=pOptimizer;
}
void Tracking::setLocalMapper(LocalMapping* pLocalMapper)
{
  mpLocalMapper=pLocalMapper;
}

void Tracking::Run()
{
  eState=NOT_INITIALIZED;
  while(1)
  {
    if(CheckNodes())
    {
      processNode();
   //   trackNode();
    }
    //TODO:reset or not 
    //TODO:finish or not 
    
    //TODO:more or less
    usleep(5000);
  }
}
void Tracking::InsertNode(Node* pNode)
{
  std::unique_lock<std::mutex> lock(mMutexTrackQueue);
  mpTrackQueue.push_back(pNode);
}
bool Tracking::CheckNodes()
{
  std::unique_lock<std::mutex> lock(mMutexTrackQueue);
  return (!mpTrackQueue.empty());
}
bool Tracking::processNode()
{
  {
    std::unique_lock<std::mutex> lock(mMutexTrackQueue);
    mpCurrNode=mpTrackQueue.front();
    mpTrackQueue.pop_front();
  }
  if(eState==NOT_INITIALIZED)
  {
     firstNode();
     eState=TRACK_OK;
     return true;
  }
  return otherNode();
}
void Tracking::firstNode()
{
  mpCurrNode->id_=graph_.size();
  mpCurrNode->seq_id_=next_vertex_id++;
  mpCurrNode->vertex_id_=next_vertex_id++;
  graph_[mpCurrNode->id_]=mpCurrNode;
  
  keyframe_ids_.push_back(mpCurrNode->id_);
  
  mpOptimizer->InsertFirstNode(mpCurrNode);
}
bool Tracking::otherNode()
{
  Eigen::Matrix4d motion_estimate;
  bool edge_to_last_keyframe_found=false;
  bool found_match=nodeComparisons(mpCurrNode,motion_estimate,edge_to_last_keyframe_found);
  
  mpLocalMapper->InsertNode(mpCurrNode);
}
bool Tracking::nodeComparisons(Node* new_node, Eigen::Matrix4d& curr_motion_estimate, bool& edge_to_keyframe)
{
  new_node->id_=graph_.size();
  new_node->seq_id_=next_seq_id++;
  
  //Initial Comparison
  MatchingResult mr;
  Node* prev_node=graph_[graph_.size()-1];
  mr=new_node->matchNodePair(prev_node);
  
  if(mr.edge.id1>=0&&mr.edge.id2>=0)
  {
      double time1=prev_node->timestamp_;
      double time2=new_node->timestamp_;
      double delta_time=time2-time1;
      //TODO:make sure the unit of delta_time is second;
      if(!isBigTrafo(mr.edge.transform)||isSmallTrafo(mr.edge.transform,delta_time))
      {
	//Track lost 
	eState=TRACK_LOST;
	return false;
      }
      else//Good Transformation 
      {
	 //TODO:the new edge could add to the pose graph or not
	 if()
	 {
	   graph_[new_node->id_]=new_node;
	   if(keyframe_ids_.contains(mr.edge.id1))
	     edge_to_keyframe=true;
	 }
	 else
	 {
	    eState=TRACK_LOST;
	    return false;
	 }
      }
  }
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
 /* std::list<MatchingResult> results;
  for(std::list<const Node*>::iterator it=nodes_to_comp.begin();it!=nodes_to_comp.end();it++)
  {
    results.push_back(new_node->matchNodePair(*it));
  }
  
  for(std::list<MatchingResult>::iterator it=results.begin();it!=results.end();it++)
  {
    MatchingResult mr=*it;
  }*/
}
std::list<int> Tracking::getPotentialEdgeTargetsWithDijkstra(const Node* new_node,
    int seq_targets,int geod_targets,int samp_targets,int prodecessor_id,bool include_predecessor)
{
  std::list<int> ids_to_link_to;
  if(prodecessor_id<0)
    prodecessor_id=graph_.size()-1;
  
  //the number of vertices already in graph is not enough
  if((int)camera_vertices.size()<=seq_targets+geod_targets+samp_targets||camera_vertices.size()<=1)
  {
    seq_targets=seq_targets+geod_targets+samp_targets;
    geod_targets=0;
    samp_targets=0;
    prodecessor_id=graph_.size()-1;
  }
  
  //Time continuous
  if(seq_targets>0)
  {
    for(int i=0;i<seq_targets+1&&prodecessor_id-i>=0;i++)
    {
      ids_to_link_to.push_back(prodecessor_id-i);
    }
  }
  //spatial continuous
  if(geod_targets>0)
  {
    g2o::HyperGraph::VertexSet& vs=mpOptimizer->DijkstraFunc(graph_[prodecessor_id]->id_);
    
    std::map<int,int> vertex_id_node_id;
    for(graph_it it=graph_.begin();it!=graph_.end();++it)
    {
      Node* node=it->second;
      vertex_id_node_id[node->vertex_id_]=node->id_;
    }
  //Geodesic Neighbours except sequential
    std::map<int,int> neighbour_indices;
    int sum_of_weights=0;
    int vid,id;
    for(g2o::HyperGraph::VertexSet::iterator vit=vs.begin();vit!=vs.end();vit++)
    {
      
      vid=(*vit)->id();
      if(vertex_id_node_id.count(vid))
	id=vertex_id_node_id.at(vid);
      else
	continue;
      if(graph_.at(id)->matchable_)continue;
      if(id<prodecessor_id-seq_targets||(id>prodecessor_id&&id<=(int)graph_.size()-1))
      {
	int weight=abs(prodecessor_id-id);
	neighbour_indices[id]=weight;
	sum_of_weights+=weight;
      }
    }
  
    while(ids_to_link_to.size()<seq_targets+geod_targets&&neighbour_indices.size()!=0)
    {
      int random_pick=rand()%sum_of_weights;
      int weight_so_far=0;
      for(std::map<int,int>::iterator map_it=neighbour_indices.begin();map_it!=neighbour_indices.end();map_it++)
      {
	weight_so_far+=map_it->second;
	if(weight_so_far>random_pick)
	{
	  int sampled_id=map_it->first;
	  ids_to_link_to.push_front(sampled_id);
	  sum_of_weights-=map_it->second;
	  neighbour_indices.erase(map_it);
	  break;
	}
      }
    }
  }
  //Sample targets from graph-neighbours
  if(samp_targets>0)
  {
    std::vector<int> non_neighbour_indices;
    non_neighbour_indices.reserve(graph_.size());
    for(std::list<int>::iterator it=keyframe_ids_.begin();it!=keyframe_ids_.end();it++)
    {
      if(find(ids_to_link_to.begin(),ids_to_link_to.end(),*it)!=ids_to_link_to.end())
	continue;
      if(!graph_.at(*id)->matchable_)
	continue;
      non_neighbour_indices.push_back(*it);
    }
    while(ids_to_link_to<seq_targets+geod_targets+samp_targets&&non_neighbour_indices.size()!=0)
    {
      int id=rand()%non_neighbour_indices.size();
      int sampled_id=non_neighbour_indices[id];
      non_neighbour_indices[id]=non_neighbour_indices.back();
      non_neighbour_indices.pop_back();
      ids_to_link_to.push_front(id);
    }
  }
  if(include_predecessor)
  {
    ids_to_link_to.push_back(predecessor_id);
  }
  
  return ids_to_link_to;
}
  
}
