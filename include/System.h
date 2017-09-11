#ifndef SYSTEM_H
#define SYSTEM_h

#include <mutex>
#include <thread>
#include <string>

#include <opencv2/core/core.hpp>

#include "ParameterServer.h"

#include "NodeMaker.h"
#include "Tracking.h"
#include "LocalMapping.h"
#include "Optimizer.h"
#include "Viewer.h"

namespace ORB_RGBD_SLAM
{
class NodeMaker;
class Tracking;
class LocalMapping;
class Optimizer;
class Viewer;
class System
{
public:
  System(const std::string paramFile);
  
  void TrackRGBD(const cv::Mat& imRGB,const cv::Mat& imDepth,double timestamp);
  
private:
  NodeMaker* mpNodeMaker;
  Tracking* mpTracker;
  LocalMapping*  mpLocalMapper;
  Optimizer* mpOptimizer;
  Viewer* mpViewer;
  
  std::thread* mptTracker;
  std::thread* mptLocalMapper;
  std::thread* mptOptimizer;
  std::thread* mptViewer;
};
}
#endif