/*
 * LegOdoWrapper.hpp
 *
 *  Created on: Feb 6, 2014
 *      Author: dehann
 */

#ifndef LEGODOWRAPPER_HPP_
#define LEGODOWRAPPER_HPP_

#include <path_util/path_util.h>
#include <estimate/leg_odometry.hpp>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

struct CommandLineConfig
{
    std::string param_file;
    std::string urdf_file;
    std::string in_log_name;
    std::string out_log_name;
    bool read_lcmlog;
    int64_t begin_timestamp;
    int64_t end_timestamp;
    bool republish_incoming;
    int processing_rate;
};

// Internal state and LegOdo logic wrapper used by App and StateEstimator classes
class LegOdoWrapper {
public:
  LegOdoWrapper(boost::shared_ptr<lcm::LCM> &lcm_subscribe_, boost::shared_ptr<lcm::LCM> &lcm_publish_, CommandLineConfig &cl_cfg__) :
		lcm_subscribe_(lcm_subscribe_), lcm_publish_(lcm_publish_), cl_cfg_(cl_cfg__) { }

  void setupLegOdo();

protected:
  boost::shared_ptr<lcm::LCM> lcm_subscribe_, lcm_publish_;
  BotParam* botparam_;
  boost::shared_ptr<ModelClient> model_;

  CommandLineConfig cl_cfg_;
  leg_odometry* leg_odo_;

  // logging:
  BotFrames* frames_;
  bot::frames* frames_cpp_;
  //void openLogFile();
  //std::ofstream logfile_;
  //void terminate();

};

// Use App if you want a stand-alone, LCM listening application for doing leg odometry
class App : public LegOdoWrapper {
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_subscribe_, boost::shared_ptr<lcm::LCM> &lcm_publish_, CommandLineConfig& cl_cfg_);
    ~App();

  private:
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);
};



#endif /* LEGODOWRAPPER_HPP_ */
