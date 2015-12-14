#ifndef CONVERT_OCTOMAP_HPP_
#define CONVERT_OCTOMAP_HPP_

// file i-o
#include <sys/types.h>
#include <sys/stat.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <octomap/octomap.h>
#include <octomap_utils/octomap_util.hpp>
#include <lcmtypes/octomap_utils.h>

#include <path_util/path_util.h>
#include <pronto_utils/pronto_vis.hpp>

using namespace std;
using namespace octomap;

struct ConvertOctomapConfig
{
    double octomap_resolution;
    double blur_sigma;
    bool blur_map;
};


class ConvertOctomap{
  public:
    ConvertOctomap(boost::shared_ptr<lcm::LCM> &lcm_, const ConvertOctomapConfig& co_cfg_);
    
    ~ConvertOctomap(){
    }    

    void doWork(pronto::PointCloud* &cloud);
    void doWork(pronto::PointCloud* &cloud, string octree_channel);
    OcTree* getTree(){ return tree_; }
    bool clearTree(){ 
      tree_->clear();
      return true; 
    }
    void publishOctree(OcTree* tree, std::string channel);
    
  private:

    OcTree* convertPointCloudToOctree(pronto::PointCloud* &cloud);
    void convertCloudAndUpdateOctree(pronto::PointCloud* &cloud, octomap::OcTree& tree);

    pronto_vis* pc_vis_ ;

    OcTree* tree_;    

    boost::shared_ptr<lcm::LCM> lcm_;
    
    const ConvertOctomapConfig co_cfg_;   
    
    int verbose_;
};


#endif
