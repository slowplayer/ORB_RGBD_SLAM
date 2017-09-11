#include "Viewer.h"

namespace ORB_RGBD_SLAM
{
Viewer::Viewer()
{

}
void Viewer::Run()
{
  pangolin::CreateWindowAndBind("ORB_RGBD_SLAM",1024,768);
  
  glEnable(GL_DEPTH_TEST);
  
  glEnable(GL_BLEND);
  glBlendColor(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  
  pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
    pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ,0,0,0,0.0,-1.0,0.0);
  pangolin::View& d_cam=pangolin::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0).SetHandler(new pangolin::Handler3D(s_cam));
  
 
}
}