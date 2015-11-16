#ifndef CLOUD_ACCUMULATE_HPP_
#define CLOUD_ACCUMULATE_HPP_

#include <deque>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/pronto/pointcloud2_t.hpp>

#include <laser_utils/laser_util.h>
#include <path_util/path_util.h>

//#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pronto_utils/pronto_vis.hpp>
#include <pronto_utils/pronto_lcm.hpp> // decode perception lcm messages
#include <pronto_utils/pronto_frame_check_tools.hpp>
////////////////////////////////////////
struct CloudAccumulateConfig
{
    int batch_size;
    std::string lidar_channel;
    double max_range;
    double min_range;
};

class CloudAccumulate{
  public:
    CloudAccumulate(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_);
    
    ~CloudAccumulate(){
    }    
    
    int getCounter(){ return counter_; }
    bool getFinished(){ return finished_; }
    pronto::PointCloud* getCloud(){ return combined_cloud_; }
    
    void clearCloud(){ 
      combined_cloud_->points.clear(); 
      counter_ = 0;
      finished_ = false;
      std::cout << "Empty previous map\n";
    }
    
    void publishCloud(pronto::PointCloud* &cloud);
    void processLidar(const  bot_core::planar_lidar_t* msg);
    void processVelodyne(const  pronto::pointcloud2_t* msg);

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CloudAccumulateConfig& ca_cfg_;
    
    
    
    pronto_vis* pc_vis_ ;
    BotParam* botparam_;
    BotFrames* botframes_;
    FrameCheckTools frame_check_tools_;
    
    int counter_; 
    int verbose_;
    
    pronto::PointCloud* combined_cloud_;
    
    bool finished_;
    
    Laser_projector * laser_projector_;
    laser_projected_scan * projected_laser_scan_;  
    
    pronto::PointCloud* convertPlanarScanToCloud(std::shared_ptr<bot_core::planar_lidar_t> this_msg);
    
};


#endif
