#ifndef CONVERT_OCTOMAP_HPP_
#define CONVERT_OCTOMAP_HPP_

// file i-o
#include <sys/types.h>
#include <sys/stat.h>


#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap_utils/octomap_util.hpp>
#include <lcmtypes/octomap_utils.h>

#include <path_util/path_util.h>

using namespace std;
using namespace octomap;

struct ConvertOctomapConfig
{
    double blur_sigma;
    bool blur_map;
};


class ConvertOctomap{
  public:
    ConvertOctomap(boost::shared_ptr<lcm::LCM> &lcm_, const ConvertOctomapConfig& co_cfg_);
    
    ~ConvertOctomap(){
    }    

    void doWork(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    OcTree* getTree(){ return tree_; }
    void publishOctree(OcTree* tree, std::string channel);
    
  private:

    OcTree* convertPointCloudToOctree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    
    OcTree* tree_;    

    boost::shared_ptr<lcm::LCM> lcm_;
    
    const ConvertOctomapConfig co_cfg_;   
    
    int verbose_;
};


#endif
