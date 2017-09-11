#ifndef  NODE_MAKER_H
#define  NODE_MAKER_H

#include "Node.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "features.h"

namespace ORB_RGBD_SLAM
{
class Tracking;
class NodeMaker
{
public:
  NodeMaker();
  
  void GrabRGBD(const cv::Mat& imRGB,const cv::Mat& imDepth,double timestamp);
  
  void setTracker(Tracking* pTracker);
private:
  void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);
  
  //feature detector and descriptor extractor
  cv::Ptr<cv::Feature2D> detector_;
  cv::cv::Ptr<cv::DescriptorExtractor> extractor_;
  
  Tracking* mpTracker;
};
}
#endif