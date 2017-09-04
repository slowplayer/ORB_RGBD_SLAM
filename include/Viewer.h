#ifndef VIEWER_H
#define VIEWER_H

#include <pangolin/pangolin.h>

#include "MapDrawer.h"
#include "FrameDrawer.h"

namespace ORB_RGBD_SLAM
{
class MapDrawer;
class FrameDrawer;
class Viewer
{
public:
  Viewer(FrameDrawer* pFrameDrawer,MapDrawer* pMapDrawer);
  void Run();
private:
  float mViewpointX,mViewpointY,mViewpointZ,mViewpointF;
};
}
#endif