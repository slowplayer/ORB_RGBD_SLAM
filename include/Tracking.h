#ifndef TRACKING_H
#define TRACKING_H

#include <list>
#include <mutex>

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
  
  std::list<Node*> mpTrackQueue;
  std::mutex mMutexTrackQueue;
  
  Optimizer* mpOptimizer;
  LocalMapping* mpLocalMapper;
  
  
  Node* mpCurrNode;
};
}
#endif