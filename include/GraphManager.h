#ifndef GRAPH_MANAGER_H
#define GRAPH_MANAGER_H

#include <list>
#include <map>

#include <Eigen/Core>

#include "Node.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_levenberg.h"




namespace ORB_RGBD_SLAM
{
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
typedef g2o::LinearSolverDense<SlamBlockSolver::PoseMatrixType> SlamLinearDenseSolver;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
typedef std::pair<int, g2o::HyperGraph::Vertex*> VertexIDPair;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

class Node;
class GraphManager
{
public:
  GraphManager();
  
  bool addNode(Node* new_node);
private:
  void firstNode(Node* new_node);
  bool otherNode(Node* new_node);
  
  bool nodeComparisons(Node* new_node,
		       Eigen::Matrix4f& curr_motion_estimate,
		       bool& edge_to_keyframe);
  
  void optimizeGraph();
  
  void addKeyframe(int id);
  
  std::map<int,Node*> graph_;
  std::list<int> keyframe_ids_;
  unsigned int min_matches;
  unsigned int next_seq_id,next_vertex_id;
};
}






#endif