#include "LocalMapping.h"

namespace ORB_RGBD_SLAM
{
LocalMapping::LocalMapping()
{

}
void LocalMapping::InsertNode(Node* pNode,std::list<const Node*>& pComp)
{
  {
    std::unique_lock<std::mutex> lock(mMutexMapQueue);
    mpMapQueue.push_back(pNode);
    mpCompQueue.push_back(pComp);
  }
}

bool LocalMapping::CheckNodes()
{
  std::unique_lock<std::mutex> lock(mMutexMapQueue);
  return (!mpMapQueue.empty()&&!mpCompQueue.empty());
}

void LocalMapping::Run()
{

}

}