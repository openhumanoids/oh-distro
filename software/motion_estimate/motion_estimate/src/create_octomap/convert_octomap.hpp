#ifndef CONVERT_OCTOMAP_HPP_
#define CONVERT_OCTOMAP_HPP_


#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap_utils/octomap_util.hpp>
#include <lcmtypes/octomap_utils.h>

using namespace std;
using namespace octomap;

struct ConvertOctomapConfig
{
    double blur_sigma;
    double repeat_period;
    bool write_output;
    bool blur_map;
};


class ConvertOctomap{
  public:
    ConvertOctomap(boost::shared_ptr<lcm::LCM> &lcm_, const ConvertOctomapConfig& co_cfg_);
    
    ~ConvertOctomap(){
    }    

    void doWork(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    
  private:

    OcTree* convertPointCloudToOctree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    void publishOctree(OcTree* tree, std::string channel);
    
    
    boost::shared_ptr<lcm::LCM> lcm_;
    
    const ConvertOctomapConfig co_cfg_;   
    
    int verbose_;
};


#endif