#include "Tracker.h"

namespace ORB_RGBD_SLAM
{
Tracker::Tracker()
{
  detector_=createDetector();
  extractor_=createDescriptorExtractor();
  
  //TODO: fix GraphManager constructor
  graph_mgr=new GraphManager();
}
void Tracker::GrabRGBD(const cv::Mat& imRGB, const cv::Mat& imDepth, double timestamp)
{
  cv::Mat gray_img;
  cv::Mat depth_mono8_img;
  
  cv::cvtColor(imRGB,gray_img,CV_RGB2GRAY);
  
  //TODO:make sure the type of imDepth is CV_32FC1
  depthToCV8UC1(imDepth,depth_mono8_img);
  
  Node* node_ptr=new Node(gray_img,imDepth,depth_mono8_img,timestamp,detector_,extractor_);
  
  processNode(node_ptr);
}
void Tracker::depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img)
{
  //Process images
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
void Tracker::processNode(Node* new_node)
{

}
}