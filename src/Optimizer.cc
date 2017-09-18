#include "Optimizer.h"

namespace ORB_RGBD_SLAM
{
Optimizer::Optimizer():
optimizer_(NULL),
next_seq_id(0),next_vertex_id(0)
{
    createOptimizer();
}
void Optimizer::InsertFirstNode(Node* pNode)
{
   pNode->id_=graph_.size();
   pNode->seq_id_=next_vertex_id++;
   pNode->vertex_id_=next_vertex_id++;
   graph_[pNode->id_]=pNode;
   
   init_base_pose_=Eigen::Matrix4d::Identity();
   
   g2o::VertexSE3* reference_pose=new g2o::VertexSE3;
   reference_pose->setId(pNode->vertex_id_);
   camera_vertices.insert(reference_pose);
   
   g2o::SE3Quat g2o_ref_se3=eigen2G2O(init_base_pose_);
   reference_pose->setEstimate(g2o_ref_se3);
   reference_pose->setFixed(true);
   
   addVertex(reference_pose);
   
   //TODO:about the first keyframe
}
void Optimizer::addVertex(g2o::VertexSE3* pose)
{
  std::unique_lock<std::mutex> lock(mMutexOptimizer);
  optimizer_->addVertex(pose);
}

void Optimizer::InsertOtherNode(Node* pNode)
{
  std::unique_lock<std::mutex> lock(mMutexOptimizeQueue);
  mpOptimizeQueue.push_back(pNode);
}
bool Optimizer::CheckNodes()
{
  std::unique_lock<std::mutex> lock(mMutexOptimizeQueue);
  return (!mpOptimizeQueue.empty());
}

void Optimizer::createOptimizer()
{
  if(optimizer_!=NULL)
  {
    optimizer_->edges().clear();
    optimizer_->vertices().clear();
    delete optimizer_;
  }
  optimizer_=new g2o::SparseOptimizer();
  optimizer_->setVerbose(false);
  
  SlamBlockSolver* solver=NULL;
  SlamLinearDenseSolver* linearSolver=new SlamLinearDenseSolver();
  solver=new SlamBlockSolver(linearSolver);
  
  g2o::OptimizationAlgorithmLevenberg* algo=new
  g2o::OptimizationAlgorithmLevenberg(solver);
  optimizer_->setAlgorithm(algo);
  
}

void Optimizer::Run()
{
}
}