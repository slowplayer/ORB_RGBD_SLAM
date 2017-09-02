#ifndef ICP_SOLVER_H
#define ICP_SOLVER_H

namespace ORB_RGBD_SLAM
{
class IcpSolver 
{
public:
   IcpSolver () : 
   no_of_samples_ (0), accumulated_weight_ (0), 
   mean1_ (Eigen::Vector3f::Identity ()),
   mean2_ (Eigen::Vector3f::Identity ()),
   covariance_ (Eigen::Matrix<float, 3, 3>::Identity ())
   { reset (); }
   ~IcpSolver () { };
   inline void reset ();
   inline float getAccumulatedWeight () const { return accumulated_weight_;}
   inline unsigned int getNoOfSamples () { return no_of_samples_;}
   inline void add (const Eigen::Vector3f& point, const Eigen::Vector3f& corresponding_point, float weight=1.0);
   inline Eigen::Affine3f getTransformation ();
protected:
   unsigned int no_of_samples_;
   float accumulated_weight_;
   Eigen::Vector3f mean1_, mean2_;
   Eigen::Matrix<float, 3, 3> covariance_;
};
}
#endif