#include "NodeMaker.h"

namespace ORB_RGBD_SLAM
{
NodeMaker::NodeMaker()
{
  detector_=createDetector();
  extractor_=createDescriptorExtractor();
}
void NodeMaker::setTracker(Tracking* pTracker)
{
  mpTracker=pTracker;
}
void NodeMaker::GrabRGBD(const cv::Mat& imRGB, const cv::Mat& imDepth, double timestamp)
{
  cv::Mat depth_mono8_img; 
  
  depthToCV8UC1(imDepth,depth_mono8_img);
  
  Node* node_ptr=new Node(imRGB,imDepth,depth_mono8_img,timestamp,detector_,extractor_);
  
  //TODO:add node to Tracker queue
  mpTracker->InsertNode(node_ptr);
}
void NodeMaker::depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img)
{
  if(depth_img.type() == CV_32FC1)
  {
    depth_img.convertTo(mono8_img, CV_8UC1, 100,0); //milimeter (scale of mono8_img does not matter)
  }
  else if(depth_img.type() == CV_16UC1)
  {
    mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
    cv::Mat float_img;
    depth_img.convertTo(mono8_img, CV_8UC1, 0.05, -25); //scale to 2cm
    depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
    depth_img = float_img;
  }
}
}
