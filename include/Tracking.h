#ifndef TRACKING_H
#define TRACKING_H

#include <list>
#include <mutex>
#include <map>

#include "Node.h"

#include "Optimizer.h"
#include "LocalMapping.h"

namespace ORB_RGBD_SLAM
{
class Optimizer;
class LocalMapping;
class Tracking
{
public:
  Tracking();
  
  void Run();
  
  void setOptimizer(Optimizer* pOptimizer);
  void setLocalMapper(LocalMapping* pLocalMapper);
  void InsertNode(Node* pNode);
  
  enum eTrackingState
  {
    SYSTEM_NOT_READY=-1,
    NOT_INITIALIZED=0,
    TRACK_OK=1,
    TRACK_LOST=2
  };
  eTrackingState eState;
private:
  bool CheckNodes();
  bool processNode();

  void firstNode();
  bool otherNode();
  
  bool nodeComparisons(Node* new_node,
    Eigen::Matrix4d& curr_motion_estimate,bool& edge_to_keyframe);	
  std::list<int> getPotentialEdgeTargetsWithDijkstra(const Node* new_node,
    int seq_targets,int geod_targets,int samp_targets,int prodecessor_id,bool include_predecessor);
  
  
  std::list<Node*> mpTrackQueue;
  std::mutex mMutexTrackQueue;
  
  Optimizer* mpOptimizer;
  LocalMapping* mpLocalMapper;
  
  unsigned int next_seq_id;
  unsigned int next_vertex_id;
  std::map<int,Node*> graph_;
  
  std::list<int> keyframe_ids_;
  
  Node* mpCurrNode;
};
}
#endif