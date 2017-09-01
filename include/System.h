#ifndef SYSTEM_H
#define SYSTEM_h

#include <string>
#include <opencv2/core/core.hpp>

#include "ParameterServer.h"

#include "Tracker.h"
#include "Viewer.h"

namespace ORB_RGBD_SLAM
{
class System
{
public:
  System(const std::string paramFile);
  
  void TrackRGBD(const cv::Mat& imRGB,const cv::Mat& imDepth,double timestamp);
  
private:
  Viewer* mpViewer;
  Tracker* mpTracker;
};
}
#endif