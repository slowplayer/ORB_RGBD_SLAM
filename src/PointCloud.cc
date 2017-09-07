#include "PointCloud.h"

namespace ORB_RGBD_SLAM 
{
PointCloud::PointCloud(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
:x_(x),y_(y),z_(z),r_(r),g_(g),b_(b)
{

}
PointCloud::PointCloud(const PointCloud& p)
:x_(p.x()),y_(p.y()),z_(p.z()),r_(p.r()),g_(p.g()),b_(p.b())
{
  
}
}