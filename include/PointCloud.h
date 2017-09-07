#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

namespace ORB_RGBD_SLAM
{
class PointCloud
{
public:
  PointCloud(float x,float y,float z,uint8_t r,uint8_t g,uint8_t b);
  PointCloud(const PointCloud& pointcloud);
  
  inline float x(){return x_;}
  inline float y(){return y_;}
  inline float z(){return z_;}
  
  inline uint8_t r(){return r_;}
  inline uint8_t g(){return g_;}
  inline uint8_t b(){return b_;}
private:
  float x_,y_,z_;
  uint8_t r_,g_,b_;
};
}
#endif