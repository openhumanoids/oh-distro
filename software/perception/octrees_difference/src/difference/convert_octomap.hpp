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

#include <octomap/ColorOcTree.h>

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
    ColorOcTree* getTree(){ return tree_; }
    bool clearTree(){ 
      tree_->clear();
      return true; 
    }
    void publishOctree(ColorOcTree* tree, std::string channel);
    void printChangesByColor(ColorOcTree& tree);
    void printChangesAndActual(ColorOcTree& tree);
    void colorChanges(ColorOcTree& tree, int idx); //idx is an index representing 
                                                   //the current cloud, either 0 or 1
    
  private:

    void updateOctree(pronto::PointCloud* &cloud, octomap::ColorOcTree* tree);
    ScanGraph* convertPointCloudToScanGraph(pronto::PointCloud* &cloud);

    pronto_vis* pc_vis_ ;
    boost::shared_ptr<lcm::LCM> lcm_;

    ColorOcTree* tree_;    
    
    const ConvertOctomapConfig co_cfg_;   
    
    int verbose_;

    //Colors for change detection
    ColorOcTreeNode::Color* yellow;
    ColorOcTreeNode::Color* blue;
    ColorOcTreeNode::Color* green;
};


#endif
