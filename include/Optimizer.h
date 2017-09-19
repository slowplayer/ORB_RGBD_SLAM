#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <map>
#include <set>
#include <list>
#include <mutex>

#include "tools.h"

#include "Node.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

namespace ORB_RGBD_SLAM
{
typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3> > SlamBlockSolver;
typedef g2o::LinearSolverDense<SlamBlockSolver::PoseMatrixType> SlamLinearDenseSolver;

typedef std::trl::unordered_map<int,g2o::HyperGraph::Vertex*> VertexIDMap;
typedef std::pair<int,g2o::HyperGraph::Vertex*> VertexIDPair;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;
  
class Optimizer
{
public:
  Optimizer();
  
  void InsertFirstNode(Node* pNode);
  void InsertOtherNode(Node* pNode);
  
  void Run();
  
  g2o::HyperGraph::VertexSet& DijkstraFunc(int id);
  
  //TODO:may need lock
  g2o::HyperGraph::VertexSet camera_vertices;
  g2o::HyperGraph::EdgeSet cam_cam_edges_;
  g2o::HyperGraph::EdgeSet current_match_edges_;
private:
  void createOptimizer();
  bool CheckNodes();
  
  void addVertex(g2o::VertexSE3* pose);
  
  Eigen::Matrix4d init_base_pose_;
  
  std::mutex mMutexOptimizer;
  g2o::SparseOptimizer* optimizer_;
  
  std::list<Node*> mpOptimizeQueue;
  std::mutex mMutexOptimizeQueue;
  
};
}
#endif