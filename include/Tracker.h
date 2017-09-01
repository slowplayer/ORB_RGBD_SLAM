#ifndef TRACKER_H
#define TRACKER_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "features.h"


#include "GraphManager.h"
#include "Node.h"

namespace ORB_RGBD_SLAM
{
class GraphManager;
class Node;
class Tracker
{
public:
   Tracker();
   
   void GrabRGBD(const cv::Mat& imRGB,const cv::Mat& imDepth,double timestamp);
private:
    void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);
    void processNode(Node* new_node);
  
    GraphManager* graph_mgr;
    
    cv::Ptr<cv::Feature2D> detector_;
    cv::cv::Ptr<cv::DescriptorExtractor> extractor_;
};
}
#endif