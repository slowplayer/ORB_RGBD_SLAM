#ifndef LOCAL_MAPPING_H
#define LOCAL_MAPPING_H

#include <list>
#include <mutex>

#include "Node.h"
#include "Optimizer.h"

namespace ORB_RGBD_SLAM
{
class Optimizer;
class LocalMapping
{
public:
  LocalMapping();
  
  void setOptimizer(Optimizer* pOptimizer);
  void InsertNode(Node* pNode,std::list<const Node*>& pComp);
  void addKeyframe(int id);
  void Run();
private:
  bool CheckNodes();
  
  std::list<Node*> mpMapQueue;
  std::list<std::list<const Node*> > mpCompQueue;
  std::mutex mMutexMapQueue;
  
  Optimizer* mpOptimizer;
};
}
#endif