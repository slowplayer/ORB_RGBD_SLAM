#include "Tracking.h"

namespace ORB_RGBD_SLAM
{
Tracking::Tracking()
:eState(SYSTEM_NOT_READY)
{

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
  //TODO:FIRST NODE?
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
     return firstNode();
  }
  return otherNode();
}

}
