#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <bot_core/bot_core.h>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <pointcloud_tools/pointcloud_math.hpp>

#include <ConciseArgs>

using namespace std;


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, int stage_, int mode_, int footsteps_);
    
    ~Pass(){
    }    
        
  private:
    int verbose_;
    boost::shared_ptr<lcm::LCM> lcm_;
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
    void candidateFootstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::footstep_plan_t* msg);    
    void candidateRobotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_t* msg);    
    void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::controller_status_t* msg);    
    
    void sendDataRequests(int64_t this_utime);
    
    Eigen::Isometry3d getRandomGoal();
    int stage_;
    int mode_;
    std::vector<std::string> mode_names_;
    int test_counter_;
    
    int footsteps_;
    
    drc::footstep_plan_t last_footstep_msg_;
    
    bool was_walking_last_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, int stage_, int mode_, int footsteps_): 
    lcm_(lcm_), stage_(stage_), mode_(mode_), footsteps_(footsteps_){
  

  lcm_->subscribe( "EST_ROBOT_STATE" ,&Pass::robotStateHandler,this);
  lcm_->subscribe("CANDIDATE_FOOTSTEP_PLAN",&Pass::candidateFootstepPlanHandler,this);   
  lcm_->subscribe("CANDIDATE_ROBOT_PLAN",&Pass::candidateRobotPlanHandler,this);   
  lcm_->subscribe("CONTROLLER_STATUS",&Pass::controllerStatusHandler,this);   
  srand (time(NULL)); // random seed  
  test_counter_=0;
  
  mode_names_.push_back("straight");
  mode_names_.push_back("left");
  mode_names_.push_back("right");
  
  // Initial request for height map
  sendDataRequests(0);
  sleep(1);
  
  was_walking_last_=false;
}


// get the robots position and rotation in rpy
Eigen::Isometry3d Pass::getRandomGoal(){
    float dx=0.0;
    float dy=0.0;
    float dyaw=0.0;
    
    // plus or minus
    float nx=1.0;
    float ny=1.0;
    float nyaw = 0.5;  // was 1
    if (mode_==0){
      dx = 3.0 + (float)  nx*2*( ((float) rand ()/(RAND_MAX)) -0.5);
      dy = 0.0 + (float)  ny*2*( ((float) rand ()/(RAND_MAX)) -0.5);
      dyaw = 0.0 + (float) nyaw*2*( ((float) rand ()/(RAND_MAX)) -0.5); // -/+28degrees
    }else if(mode_==1){
      dx = 3.0 + (float)  nx*2*( ((float) rand ()/(RAND_MAX)) -0.5);
      dy = 3.0 + (float)  ny*2*( ((float) rand ()/(RAND_MAX)) -0.5);
      dyaw = 0.75 + (float)  nyaw*2*( ((float) rand ()/(RAND_MAX)) -0.5);
    }else if(mode_==2){
      dx =  3.0 + (float)  nx*2*( ((float) rand ()/(RAND_MAX)) -0.5);
      dy = -3.0 + (float)  ny*2*( ((float) rand ()/(RAND_MAX)) -0.5);
      dyaw = -0.75 + (float)  nyaw*( ((float) rand ()/(RAND_MAX)) -0.5);
    }else if(mode_==3){ // this doesnt work - too random
      dx =   100.0*( ((float) rand ()/(RAND_MAX)) -0.5);
      dy =   100.0*( ((float) rand ()/(RAND_MAX)) -0.5);
      dyaw = 100.0 + (float)  nyaw*( ((float) rand ()/(RAND_MAX)) -0.5);
    }

    Eigen::Isometry3d delta_pose;
    delta_pose.setIdentity();
    delta_pose.translation()  << dx, dy, 0.0;
    delta_pose.rotate( euler_to_quat(dyaw, 0,0) );    
    
    std::cout << "Stage 0: DX: " << dx << " DY " << dy << " DYAW: " << 180.0*dyaw/M_PI <<"\n";   
    return delta_pose;    
}



void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::robot_state_t* msg){
  if (msg->utime < 5*1E6){
    sendDataRequests(msg->utime);
  }
  if (stage_==0){
    sendDataRequests(msg->utime);

    Eigen::Isometry3d current_pose;
    current_pose.setIdentity();
    current_pose.translation()  << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
    Eigen::Quaterniond quat_in = Eigen::Quaterniond( msg->origin_position.rotation.w, msg->origin_position.rotation.x,
                     msg->origin_position.rotation.y, msg->origin_position.rotation.z);
    current_pose.rotate(quat_in);    
    Eigen::Isometry3d goal_pose = current_pose*getRandomGoal();
    
    ////////////////////////////////
    drc::walking_goal_t goal;
    goal.utime  = msg->utime;
    goal.robot_name = "atlas";
    drc::position_3d_t goal_pos;
    drc::vector_3d_t trans;// = drc_vector_3d_t();
    trans.x =  goal_pose.translation().x();
    trans.y =  goal_pose.translation().y();
    trans.z = 0.0;
    goal_pos.translation = trans;
    drc::quaternion_t quat;
    Eigen::Quaterniond quat_out = Eigen::Quaterniond(goal_pose.rotation());
    quat.w = quat_out.w();
    quat.x = quat_out.x();
    quat.y = quat_out.y();
    quat.z = quat_out.z();
    goal_pos.rotation = quat;
    goal.goal_pos = goal_pos;
    goal.max_num_steps = footsteps_;
    goal.min_num_steps = 0;
    goal.timeout = 0;
    goal.step_speed = 0.5;
    goal.step_height = 0.05;
    goal.follow_spline = true;
    goal.ignore_terrain = false;
    goal.mu = 1.0;
    goal.allow_optimization = false;
    goal.is_new_goal = true;
    goal.right_foot_lead = true;
    lcm_->publish("WALKING_GOAL", &goal);
   
    sleep(5); // added to make sure that the footstep plane is new before accepting it, could be smaller?
    stage_=1;
    std::cout << "Stage 0: got EST_ROBOT_STATE, publishing WALKING_GOAL. Moving to Stage 1\n";
    std::cout << "         Mode: " << mode_names_[mode_] << " | Sequence: " << test_counter_ <<"\n";
    
    test_counter_ ++;
    // Walk 5 steps per mode:
    bool vary_mode=false;
    if (vary_mode){
      if (test_counter_ >= 5){
	test_counter_=0;
	mode_++;
	if (mode_>2){ mode_=0; }      
	  std::cout << "         Next time the mode will be: " << mode_names_[mode_]  <<"\n";
      }
    }
  }
}

void Pass::candidateFootstepPlanHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::footstep_plan_t* msg){
  if (stage_==1){
    sendDataRequests(msg->utime);
    sleep(1); 
    lcm_->publish("APPROVED_FOOTSTEP_PLAN", msg );
    last_footstep_msg_ = *msg;
    stage_ =2;
    std::cout << "Stage 1: got CANDIDATE_FOOTSTEP_PLAN, republishing as COMMITTED. Moving to Stage 2\n";
  }
}



void Pass::candidateRobotPlanHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_plan_t* msg){
  if (stage_==2){
    sendDataRequests(msg->utime);
    sleep(1); // this was required as matlab was too slow to receive it immediately
    //lcm_->publish("COMMITTED_ROBOT_PLAN", msg );
    lcm_->publish("COMMITTED_FOOTSTEP_PLAN", &last_footstep_msg_ );
    stage_ =3;
    std::cout << "Stage 2: got CANDIDATE_ROBOT_PLAN, republishing as COMMITTED. Moving to Stage 3\n";
    std::cout << "         Sleeping for 45seconds until to allow walking to start\n";
    sleep(45);
    std::cout << "         Finished sleeping\n";
  }
}



void Pass::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::controller_status_t* msg){
  if (stage_==3){
    
    // if standing now but was walking before:
    if ((msg->state== drc::controller_status_t::STANDING)&&(was_walking_last_)){
      sendDataRequests(msg->utime);
      stage_ =0;
      std::cout << "Stage 3: controller=STANDING, restarting. Moving to Stage 0\n"
                << "===========================================================\n";
    }
    

    if (msg->state == drc::controller_status_t::WALKING){
      was_walking_last_ = true; 
    }else{
      was_walking_last_ = false;
    }
    
    
  }
}


// Send this to make sure what we need is being published for the planner:
void Pass::sendDataRequests(int64_t this_utime){
  drc::data_request_list_t reqs;
  reqs.utime = this_utime;
  drc::data_request_t req1; //EST_ROBOT_STATE
  req1.period = 10;
  req1.type = drc::data_request_t::MINIMAL_ROBOT_STATE;
  drc::data_request_t req2; // Planning Height Map
  req2.period = 10;
  req2.type = drc::data_request_t::HEIGHT_MAP_SCENE;
  reqs.requests.push_back(req1); 
  reqs.requests.push_back(req2);
  reqs.num_requests = reqs.requests.size();
  lcm_->publish("DATA_REQUEST", &reqs);
}

int main(int argc, char ** argv) {
  int verbose = 0;
  int stage = 3;
  int mode = 0;
  int footsteps = 6;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(verbose, "v", "verbosity","0 none, 1 little, 2 debug");
  opt.add(stage, "s", "stage","Starting Stage");
  opt.add(mode, "m", "mode","Mode 0forward 1left 2right");// 3random");
  opt.add(footsteps, "f", "footsteps","Number of Footsteps");// 3random");
  opt.parse();
  std::cout << "verbose: " << verbose << "\n";    
  std::cout << "stage: " << stage << "\n";    
  std::cout << "mode: " << mode << " [where 0forward 1left 2right\n";//3random]\n";    
  std::cout << "footsteps: " << footsteps << " footsteps\n";      

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, stage, mode, footsteps);
  cout << "System Test Ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
