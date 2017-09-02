#include "Node.h"

namespace ORB_RGBD_SLAM
{
Node(const cv::Mat& imGray,const cv::Mat& imDepth,
       const cv::Mat& detector_mask,double timestamp,
       cv::Ptr<cv::Feature2D> detector,
       cv::Ptr<cv::DescriptorExtractor> extractor)
:id_(-1),seq_id_(-1),vertex_id_(-1),init_node_matches_(0),timestamp_(timestamp)
{
  detector->detect(imGray,feature_location_2d_,detector_mask);
  
  removeDepthless(feature_location_2d_,imDepth);
  
  extractor->compute(imGray,feature_location_2d_,features_descriptors_);
  
  projectTo3D(feature_location_2d_,feature_location_3d_,imDepth);
  
  assert(feature_location_2d_.size()==feature_location_3d_.size());
  assert(feature_location_3d_.size()==(unsigned int)features_descriptors_.rows);
  
  
}
void Node::removeDepthless(std::vector< cv::KeyPoint >& feature_location_2d, const cv::Mat& depth)
{
  cv::Point2f  p2d;
  float Z;
  unsigned int i=0;
  while(i<feature_location_2d.size())
  {
    p2d=feature_location_2d[i].pt;
    if(p2d.x>=depth.cols||p2d.x<0||
      p2d.y>=depth.rows||p2d.y<0||
      std::isnan(p2d.x)||std::isnan(p2d.y))
    {
      feature_location_2d.erase(feature_location_2d.begin()+i);
      continue;
    }
    Z=depth.at<float>(round(p2d.y),round(p2d.x));
    if(std::isnan(Z))
    {
      feature_location_2d.erase(feature_location_2d.begin()+i);
      continue;
    }
    i++;
  }
  
  size_t max_keyp=static_cast<size_t>(ParameterServer::instance()->getParam("max_keypoints"));
  if(feature_location_2d.size()>max_keyp)
  {
    cv::KeyPointsFilter::retainBest(feature_location_2d,max_keyp);
    feature_location_2d.resize(max_keyp);
  }
}

void Node::projectTo3D(std::vector< cv::KeyPoint >& feature_location_2d, std_vector_of_eigen_vector4f& feature_location_3d, const cv::Mat& depth)
{
  ParameterServer* ps=ParameterServer::instance();
  
  double depth_scaling=static_cast<double>(ps->getParam("depth_scaling_factor"));
  size_t max_keyp=static_cast<size_t>(ps->getParam("max_keypoints"));
  double maximum_depth=static_cast<double>(ps->getParam("maximum_depth"));
  float fxinv=1./ps->getParam("camera_fx");
  float fyinv=1./ps->getParam("camera_fy");
  float cx=ps->getParam("camera_cx");
  float cy=ps->getParam("camera_cy");
  
  if(feature_location_3d.size())
    feature_location_3d.clear();
  
  cv::Point2d p2d;
  float Z;
  float x,y,z;
  for(unsigned int i=0;i<feature_location_2d.size();i++)
  {
    Z=depth.at<float>(round(p2d.y),round(p2d.x));
    
    x=(p2d.x-cx)*Z*fxinv;
    y=(p2d.y-cy)*Z*fyinv;
    z=Z;
    
    feature_location_3d.push_back(Eigen::Vector4f(x,y,z,1.0));
  }
}

MatchingResult Node::matchNodePair(const Node* older_node)
{
  MatchingResult mr;
  bool found_transformation=false;
  double ransac_quality=0;
  unsigned int min_matches=(unsigned int)ParameterServer::instance()->getParam("min_matches");
  
  featureMatching(older_node,&mr.all_matches);
  
  if(mr.all_matches.size()>=min_matches)
  {
    found_transformation=getRelativeTransformationTo(older_node,&mr->all_matches,mr.ransac_trafo,mr.rmse,mr.inlier_matches);
  }
  
}
bool isNearer(const cv::DMatch& m1, const cv::DMatch& m2) { 
  return m1.distance < m2.distance; 
}
static void keepStrongestMatches(int n, std::vector<cv::DMatch>* matches)
{
  if(matches->size() > n)
  {
    std::vector<cv::DMatch>::iterator nth = matches->begin() + n;
    std::nth_element(matches->begin(), nth, matches->end(), isNearer);
    matches->erase(nth, matches->end());
  }
}
unsigned int Node::featureMatching(const Node* other, std::vector< cv::DMatch >* matches) const
{
  assert(matches->size()==0);
  
  uint64_t* query_value=reinterpret_cast<uint64_t*>(this->features_descriptors_.data);
  uint64_t* search_array=reinterpret_cast<uint64_t*>(other->features_descriptors_.data);
  
  for(unsigned int i=0;i<this->feature_location_2d_.size();++i,query_value+=4)
  {
    int result_index=-1;
    int hd=bruteForceSearchORB(query_value,search_array,other->feature_location_2d_.size(),result_index);
    if(hd>=128)continue;
    cv::DMatch match(i,result_index,hd/256.0+(float)rand()/(1000.0*RAND_MAX));
    matches->push_back(match);
  }
  
  float max_matches=ParameterServer::instance()->getParam("max_matches");
  keepStrongestMatches(static_cast<int>(max_matches),matches);
  
  return matches->size();
}
std::vector< cv::DMatch > Node::sample_matches_prefer_by_distance(unsigned int sample_size, std::vector< cv::DMatch >& matches_with_depth)
{
  std::set<unsigned int> sampled_ids;
  int safety_net=0;
  
  while(sampled_ids.size()<sampled_size&&matches_with_depth.size()>=sample_size)
  {
      int id1=rand()%matches_with_depth.size();
      int id2=rand()%matches_with_depth.size();
      if(id1>id2)id1=id2;
      sampled_ids.insert(id1);
      if(++safety_net>10000)break;
  }
  std::vector<cv::DMatch> sampled_matches;
  sampled_matches.reserve(sampled_ids.size());
  for(std::set<unsigned int>::iterator it=sampled_ids.begin();it!=sampled_ids.end();++it)
  {
    sampled_matches.push_back(matches_with_depth[*it]);
  }
  return sampled_matches;
}
Eigen::Matrix4f getTransformFromMatches(const Node* newer_node,const Node* earlier_node,
					  const std::vector<cv::DMatch>& matches,
					  bool& valid,const float max_dist_m)
{
  IcpSolver tfc;
  valid=true;
  float weight;
  for(unsigned int i=0;i<matches.size();i++)
  {
    const cv::DMatch& m=matches[i];
    Eigen::Vector3f from=newer_node->feature_location_3d_[m.queryIdx].head<3>();
    Eigen::Vector3f to=newer_node->feature_location_3d_[m.queryIdx].head<3>();
    if(std::isnan(from(2)||std::isnan(to(2)))
      continue;
    weight=1.0/(from(2)*to(2));
    
    tfc.add(from,to,weight);
  }
  return tfc.getTransformation().matrix();
}
void Node::computeInliersAndError(const std::vector<cv::DMatch> & all_matches,
                                  const Eigen::Matrix4f& transformation4f,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  size_t min_inliers, //break if this can't be reached
                                  std::vector<cv::DMatch>& inliers, //pure output var
                                  double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                  //std::vector<double>& errors,
                                  double squaredMaxInlierDistInM) const
{ 
  inliers.clear();
  assert(all_matches.size() > 0);
  inliers.reserve(all_matches.size());
  //errors.clear();
  const size_t all_matches_size = all_matches.size();
  double mean_error = 0.0;
  Eigen::Matrix4d transformation4d = transformation4f.cast<double>();

//parallelization is detrimental here
//#pragma omp parallel for reduction (+: mean_error)
  for(int i=0; i < all_matches_size; ++i)
  //BOOST_FOREACH(const cv::DMatch& m, all_matches)
  {
    const cv::DMatch& m = all_matches[i];
    const Eigen::Vector4f& origin = origins[m.queryIdx];
    const Eigen::Vector4f& target = earlier[m.trainIdx];
    if(origin(2) == 0.0 || target(2) == 0.0){ //does NOT trigger on NaN
       continue;
    }
    double mahal_dist = errorFunction2(origin, target, transformation4d);
    if(mahal_dist > squaredMaxInlierDistInM){
      continue; //ignore outliers
    }
    if(!(mahal_dist >= 0.0)){
   //   ROS_WARN_STREAM("Mahalanobis_ML_Error: "<<mahal_dist);
   //   ROS_WARN_STREAM("Transformation for error !>= 0:\n" << transformation4d << "Matches: " << all_matches.size());
      continue;
    }
    mean_error += mahal_dist;
//#pragma omp critical
    inliers.push_back(m); //include inlier
  }


  if (inliers.size()<3){ //at least the samples should be inliers
   // ROS_DEBUG("No inliers at all in %d matches!", (int)all_matches.size()); // only warn if this checks for all initial matches
    return_mean_error = 1e9;
  } else {
    mean_error /= inliers.size();
    return_mean_error = sqrt(mean_error);
  }

}
bool Node::getRelativeTransformationTo(const Node* earlier_node, std::vector< cv::DMatch >* initial_matches, Eigen::Matrix4f& resulting_transformation, float& rmse, std::vector< cv::DMatch >& matches) const
{
  unsigned int min_inlier_threshold=static_cast<unsigned int>(ParameterServer::instance()->getParam("min_matches"));
  const float max_dist_m=ParameterServer::instance()->getParam("max_dist_for_inliers");
  const int ransac_iterations=static_cast<int>(ParameterServer::instance()->getParam("ransac_iterations"));
  const int refine_iterations=static_cast<int>(ParameterServer::instance()->getParam("refine_iterations"));
  
  double inlier_error;
  matches.clear();
  resulting_transformation=Eigen::Matrix4f::Identity();
  rmse=1e6;
  unsigned int valid_iterations=0;
  const unsigned int sample_size=4;
  bool valid_tf=false;
 
  std::vector<cv::DMatch>* matches_with_depth=initial_matches;
  std::sort(matches_with_depth->begin(),matches_with_depth->end());
  
  int real_iterations = 0;
  for(int n = 0; (n < ransac_iterations && matches_with_depth->size() >= sample_size); n++) //Without the minimum number of matches, the transformation can not be computed as usual TODO: implement monocular motion est
  {
    //Initialize Results of refinement
    double refined_error = 1e6;
    std::vector<cv::DMatch> refined_matches; 
    std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, *matches_with_depth); //initialization with random samples 
    //std::vector<cv::DMatch> inlier = sample_matches(sample_size, *matches_with_depth); //initialization with random samples 
    Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

    real_iterations++;
    for(int refinements = 1; refinements < refine_iterations; refinements++) 
    {
        Eigen::Matrix4f transformation = getTransformFromMatches(this, earlier_node, inlier,valid_tf,max_dist_m);
        if (!valid_tf || transformation!=transformation)  //Trafo Contains NaN?
          break; // valid_tf is false iff the sampled points aren't inliers themself 

        //test which features are inliers 
        computeInliersAndError(*initial_matches, transformation, 
                               this->feature_locations_3d_, //this->feature_depth_stats_, 
                               earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                               std::max(min_inlier_threshold, static_cast<unsigned int>(refined_matches.size())), //break if no chance to reach this amount of inliers
                               inlier, inlier_error, max_dist_m*max_dist_m); 
        
        if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
          break; //hopeless case
        }

        //superior to before?
        if (inlier.size() >= refined_matches.size() && inlier_error <= refined_error) {
          size_t prev_num_inliers = refined_matches.size();
          assert(inlier_error>=0);
          refined_transformation = transformation;
          refined_matches = inlier;
          refined_error = inlier_error;
          if(inlier.size() == prev_num_inliers) break; //only error improved -> no change would happen next iteration
        }
        else break;
    }  //END REFINEMENTS
    //Successful Iteration?
    if(refined_matches.size() > 0){ //Valid?
        valid_iterations++;

        //Acceptable && superior to previous iterations?
        if (refined_error <= rmse &&  
            refined_matches.size() >= matches.size() && 
            refined_matches.size() >= min_inlier_threshold)
        {
          rmse = refined_error;
          resulting_transformation = refined_transformation;
          matches.assign(refined_matches.begin(), refined_matches.end());
          //Performance hacks:
          if (refined_matches.size() > initial_matches->size()*0.5) n+=10;///Iterations with more than half of the initial_matches inlying, count tenfold
          if (refined_matches.size() > initial_matches->size()*0.75) n+=10;///Iterations with more than 3/4 of the initial_matches inlying, count twentyfold
          if (refined_matches.size() > initial_matches->size()*0.8) break; ///Can this get better anyhow?
        }
    }
  } //iterations
  if(valid_iterations == 0) // maybe no depth. Try identity?
  { 
    //IDENTITYTEST
    //1 ransac iteration with identity
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();//hypothesis
    std::vector<cv::DMatch> inlier; //result
    //test which samples are inliers 
    computeInliersAndError(*initial_matches, transformation, 
                           this->feature_locations_3d_, //this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                           min_inlier_threshold, //break if no chance to reach this amount of inliers
                           inlier, inlier_error, max_dist_m*max_dist_m); 
    
    //superior to before?
    if (inlier.size() > min_inlier_threshold && inlier_error < max_dist_m) {
      assert(inlier_error>=0);
      resulting_transformation = transformation;
      matches.assign(inlier.begin(), inlier.end());
      rmse = inlier_error;
      valid_iterations++;
    }
  } //END IDENTITY AS GUESS
  
  //TODO:G2O Refinement
  
  
  
  
  
   return matches.size()>=min_inlier_threshold;
}

  
}