#ifndef CLOUD_ACCUMULATE_HPP_
#define CLOUD_ACCUMULATE_HPP_

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <laser_utils/laser_util.h>
#include <path_util/path_util.h>

#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp>
#include <pointcloud_tools/pointcloud_lcm.hpp> // decode perception lcm messages
#include <drc_utils/frame_check_utils.hpp>
#include <drc_utils/LidarUtils.hpp>
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){ return combined_cloud_; }
    
    void clearCloud(){ 
      combined_cloud_->points.clear(); 
      counter_ = 0;
      finished_ = false;
      std::cout << "Empty previous map\n";
    }
    
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    
    void processLidar(const  bot_core::planar_lidar_t* msg);

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CloudAccumulateConfig& ca_cfg_;
    
    
    
    pointcloud_vis* pc_vis_ ;
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;  
    FrameCheckUtils frame_check_utils_;
    
    drc::LidarUtils lidar_utils_;
    
    int counter_; 
    int verbose_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud_;  
    
    bool finished_;
    
    std::deque<std::shared_ptr<bot_core::planar_lidar_t> > laser_queue_;
    
    Laser_projector * laser_projector_;
    laser_projected_scan * projected_laser_scan_;  
    
    Eigen::Isometry3d body_to_scan_last_;
    int64_t body_to_scan_last_utime_;
    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertMode1(std::shared_ptr<bot_core::planar_lidar_t> this_msg);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertMode2(std::shared_ptr<bot_core::planar_lidar_t> this_msg);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertMode3(std::shared_ptr<bot_core::planar_lidar_t> this_msg);
    
};


#endif