#ifndef TRACKING_H
#define TRACKING_H

#include <list>
#include <mutex>

#include "Node.h"

namespace ORB_RGBD_SLAM
{
class Tracking
{
public:
  Tracking();
  
  void Run();
  
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

  bool firstNode();
  bool otherNode();
  
  std::list<Node*> mpTrackQueue;
  std::mutex mMutexTrackQueue;
  
  Node* mpCurrNode;
};
}
#endif