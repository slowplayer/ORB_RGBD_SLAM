#include "System.h"

namespace ORB_RGBD_SLAM 
{
System::System(const std::string paramFile)
{
  //set config file path
  ParameterServer::instance()->setPath(paramFile);
  
  mpNodeMaker=new NodeMaker();
  mpTracker=new Tracking();
  mpLocalMapper=new LocalMapping();
  mpOptimizer=new Optimizer();
  mpViewer=new Viewer();
  
  mptTracker=new std::thread(&ORB_RGBD_SLAM::Tracking::Run,mpTracker);
  mptLocalMapper=new std::thread(&ORB_RGBD_SLAM::LocalMapping::Run,mpLocalMapper);
  mptOptimizer=new std::thread(&ORB_RGBD_SLAM::Optimizer::Run,mpOptimizer);
  mptViewer=new std::thread(&ORB_RGBD_SLAM::Viewer::Run,mpViewer);

  
  mpNodeMaker->setTracker(mpTracker);
  
}
void System::TrackRGBD(const cv::Mat& imRGB, const cv::Mat& imDepth, double timestamp)
{
  mpNodeMaker->GrabRGBD(imRGB,imDepth,timestamp);
}
}
  