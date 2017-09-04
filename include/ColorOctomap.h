#ifndef COLOR_OCTOMAP_H
#define COLOR_OCTOMAP_H

#include "ParameterServer.h"
#include "Node.h"

#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/ColorOcTree.h>

#include <thread>
#include <mutex>

#include <GL/gl.h>

namespace ORB_RGBD_SLAM
{
class ColorOctomap
{
public:
  ColorOctomap();
  ~ColorOctomap();
  
  void reset();
  
  void insertCloud(const octomap::point3d& origin,const std::vector<Eigen::Vector3d>& world_pos,const std::vector<cv::Vec3b>& feature_color_3d);
  void occupancyFilter();
  void octomapRender();
protected:
  octomap::ColorOcTree m_octoMap;
  std::thread* mptRender;
  std::mutex mMutexOctomap;
};
}

#endif