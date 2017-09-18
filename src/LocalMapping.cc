#include "LocalMapping.h"

namespace ORB_RGBD_SLAM
{
LocalMapping::LocalMapping()
{

}
void LocalMapping::InsertNode(Node* pNode)
{
  std::unique_lock<std::mutex> lock(mMutexMapQueue);
  mpMapQueue.push_back(pNode);
}

bool LocalMapping::CheckNodes()
{
  std::unique_lock<std::mutex> lock(mMutexMapQueue);
  return (!mpMapQueue.empty());
}
void LocalMapping::addKeyframe(int id)
{
  keyframe_ids_.push_back(id);
}

void LocalMapping::Run()
{

}

}