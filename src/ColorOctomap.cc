#include "ColorOctomap.h"

namespace ORB_RGBD_SLAM
{
ColorOctomap::ColorOctomap():m_octoMap(0.05)
{
  reset();
}
void ColorOctomap::reset()
{
  m_octoMap.clear();
  ParameterServer* ps=ParameterServer::instance();
  m_octoMap.setClampingThresMin(ps->getParam("octomap_clamping_min"));
  m_octoMap.setClampingThresMax(ps->getParam("octomap_clamping_max"));
  m_octoMap.setResolution(ps->getParam("octomap_resolution"));
  m_octoMap.setOccupancyThres(ps->getParam("octomap_occupancy_threshold"));
  m_octoMap.setProbHit(ps->getParam("octomap_prob_hit"));
  m_octoMap.setProbMiss(ps->getParam("octomap_prob_miss"));
}
void ColorOctomap::insertCloud(const octomap::point3d& origin, const std::vector<Eigen::Vector3d>& world_pos,const std::vector< cv::Vec3b>& feature_color_3d)
{
  Eigen::Vector3d pointWorld;
  cv::Vec3b color;
  octomap::Pointcloud cloud;
  for(std::vector<Eigen::Vector3d>::iterator it=world_pos.begin();it!=world_pos.end();it++)
  {
    pointWorld=*it;
    cloud.push_back(pointWorld[0],pointWorld[1],pointWorld[2]);
  }
  m_octoMap.insertPointCloud(cloud,origin);
  
  for(unsigned int i=0;i<feature_color_3d.size();i++)
  {
    pointWorld=world_pos[i];
    color=feature_color_3d[i];
    
    //TODO:make sure r.g.b or b.g.r
    m_octoMap.averageNodeColor(pointWorld[0],pointWorld[1],pointWorld[2],color[0],color[1],color[2]);
  }
  
  m_octoMap.updateInnerOccupancy();
}
void ColorOctomap::occupancyFilter()
{

}
void ColorOctomap::octomapRender()
{
  octomap::ColorOcTree::tree_iterator it = m_octoMap.begin_tree();
  octomap::ColorOcTree::tree_iterator end = m_octoMap.end_tree();
  int counter = 0;
  double occ_thresh = ParameterServer::instance()->getParam("occupancy_filter_threshold");
  int level = ParameterServer::instance()->getParam("octomap_display_level");
  if(occ_thresh > 0) {
    glDisable(GL_LIGHTING);
    glEnable (GL_BLEND); 
    //glDisable(GL_CULL_FACE);
    glBegin(GL_TRIANGLES);
    double stretch_factor = 128/(1 - occ_thresh); //occupancy range in which the displayed cubes can be
    for(; it != end; ++counter, ++it){
      if(level != it.getDepth()){
        continue;
      }
      double occ = it->getOccupancy();
      if(occ < occ_thresh){
        continue;
      }
      glColor4ub(it->getColor().r, it->getColor().g, it->getColor().b, 128 /*basic visibility*/ + (occ - occ_thresh) * stretch_factor );
      float halfsize = it.getSize()/2.0;
      float x = it.getX(); 
      float y = it.getY(); 
      float z = it.getZ(); 
      //Front
      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x-halfsize,y+halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z-halfsize);

      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z-halfsize);
      glVertex3f(x+halfsize,y-halfsize,z-halfsize);

      //Back
      glVertex3f(x-halfsize,y-halfsize,z+halfsize);
      glVertex3f(x+halfsize,y-halfsize,z+halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);

      glVertex3f(x-halfsize,y-halfsize,z+halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);
      glVertex3f(x-halfsize,y+halfsize,z+halfsize);

      //Left
      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x-halfsize,y-halfsize,z+halfsize);
      glVertex3f(x-halfsize,y+halfsize,z+halfsize);

      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x-halfsize,y+halfsize,z+halfsize);
      glVertex3f(x-halfsize,y+halfsize,z-halfsize);

      //Right
      glVertex3f(x+halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);

      glVertex3f(x+halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);
      glVertex3f(x+halfsize,y-halfsize,z+halfsize);

      //?
      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y-halfsize,z+halfsize);

      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y-halfsize,z+halfsize);
      glVertex3f(x-halfsize,y-halfsize,z+halfsize);

      //?
      glVertex3f(x-halfsize,y+halfsize,z-halfsize);
      glVertex3f(x-halfsize,y+halfsize,z+halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);

      glVertex3f(x-halfsize,y+halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);
      glVertex3f(x+halfsize,y+halfsize,z-halfsize);
    }
    glEnd();
  }
}

}