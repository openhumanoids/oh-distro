#ifndef octomap_trigger_hpp_
#define octomap_trigger_hpp_

#include <lcm/lcm.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>


struct Isometry3dTime{
  Isometry3dTime(int64_t utime, const Eigen::Isometry3d & pose) : utime(utime), pose(pose) {}
    int64_t utime;
    Eigen::Isometry3d pose;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

///////////////////////////////////////////////////////////////
class octomap_trigger{
  public:
    octomap_trigger(lcm_t* publish_lcm,lcm_t* subscribe_lcm);


    int initialize(int argc, char **argv);
    
    ~octomap_trigger(){
    }
    
  private:
    lcm_t* publish_lcm_;
    lcm_t* subscribe_lcm_;
    BotParam *param;
    BotFrames *frames;


    bot_core_pose_t *bot_pose_last;

    int64_t         last_velodyne_data_utime;
    int64_t           last_pose_utime;

    Isometry3dTime current_poseT;
    bool current_pose_init; // have we started
    Isometry3dTime null_poseT;
    Isometry3dTime local_poseT; // LIDAR pose where we started the most recent local map

    // Fixed transform [initally hardcoded]:
    Eigen::Isometry3d camera_to_lidar;

    // Params for new ocotomap:
    double xdim_,ydim_, zdim_;
    double res_;
    int id_;
    
    static void pose_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_pose_t* msg,
                                void* user_data) {
      ((octomap_trigger *) user_data)->pose_handler(msg);
    }
    void pose_handler(const bot_core_pose_t *msg);

};    

#endif
