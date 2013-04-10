#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <bot_core/bot_core.h>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <ConciseArgs>

using namespace std;


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
        
  private:
    int verbose_;
    boost::shared_ptr<lcm::LCM> lcm_;
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
    void candidateFootstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::footstep_plan_t* msg);    
    void candidateRobotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_plan_t* msg);    
    void controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::controller_status_t* msg);    
    int stage_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): 
    lcm_(lcm_){
  stage_=3;

  lcm_->subscribe( "EST_ROBOT_STATE" ,&Pass::robotStateHandler,this);
  lcm_->subscribe("CANDIDATE_FOOTSTEP_PLAN",&Pass::candidateFootstepPlanHandler,this);   
  lcm_->subscribe("CANDIDATE_ROBOT_PLAN",&Pass::candidateRobotPlanHandler,this);   
  lcm_->subscribe("CONTROLLER_STATUS",&Pass::controllerStatusHandler,this);   
  srand (time(NULL)); // random seed  
}


void Pass::robotStateHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::robot_state_t* msg){
  if (stage_==0){
    // q= bot_roll_pitch_yaw_to_quat (rpy)
    float dx = 3.0 + (float)  1.0*( ((float) rand ()/(RAND_MAX)) -0.5);
    float dy = 0.0 + (float)  0.5*( ((float) rand ()/(RAND_MAX)) -0.5);
    float dyaw = 0.0 + (float) 0.5*( ((float) rand ()/(RAND_MAX)) -0.5); // -/+28degrees

    double quat_in[] = {msg->origin_position.rotation.w, msg->origin_position.rotation.x,
                     msg->origin_position.rotation.y, msg->origin_position.rotation.z};
    double rpy[3];
    bot_quat_to_roll_pitch_yaw (quat_in, rpy); 
    rpy[2] = rpy[2] + dyaw;
    bot_roll_pitch_yaw_to_quat (rpy, quat_in);
    
    drc::walking_goal_t goal;
    goal.utime  = msg->utime;
    goal.robot_name = "atlas";
    drc::position_3d_t goal_pos;
    drc::vector_3d_t trans;// = drc_vector_3d_t();
    trans.x =msg->origin_position.translation.x +dx;
    trans.y =msg->origin_position.translation.y +dy; 
    trans.z = 0.0;
    goal_pos.translation = trans;
    drc::quaternion_t quat;
    quat.w = quat_in[0];
    quat.x = quat_in[1];
    quat.y = quat_in[2];
    quat.z = quat_in[3];
    goal_pos.rotation = quat;
    goal.goal_pos = goal_pos;
    goal.max_num_steps = 10;
    goal.min_num_steps = 0;
    goal.timeout = 0;
    goal.time_per_step = 100;
    goal.yaw_fixed = false;
    goal.allow_optimization = false;
    goal.is_new_goal = true;
    goal.right_foot_lead = true;
    lcm_->publish("WALKING_GOAL", &goal);
   
    stage_=1;
    std::cout << "Stage 0: DX: " << dx << " DY " << dy << " DYAW: " << 180*dyaw/M_PI <<"\n";
    std::cout << "Stage 0: got EST_ROBOT_STATE, publishing WALKING_GOAL. Moving to Stage 1\n";
  }
}

void Pass::candidateFootstepPlanHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::footstep_plan_t* msg){
  if (stage_==1){
    sleep(1); 
    lcm_->publish("COMMITTED_FOOTSTEP_PLAN", msg );
    stage_ =2;
    std::cout << "Stage 1: got CANDIDATE_FOOTSTEP_PLAN, republishing as COMMITTED. Moving to Stage 2\n";
  }
}

void Pass::candidateRobotPlanHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_plan_t* msg){
  if (stage_==2){
    sleep(1); // this was required as matlab was too slow to receive it immediately
    lcm_->publish("COMMITTED_ROBOT_PLAN", msg );
    stage_ =3;
    std::cout << "Stage 2: got CANDIDATE_ROBOT_PLAN, republishing as COMMITTED. Moving to Stage 3\n";
  }
}


void Pass::controllerStatusHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::controller_status_t* msg){
  if (stage_==3){
    if (msg->state== drc::controller_status_t::STANDING){
      stage_ =0;
      std::cout << "Stage 3: controller=STANDING, restarting. Moving to Stage 0\n"
                << "===========================================================\n";
    }
  }
}


int main(int argc, char ** argv) {
  int verbose = 0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(verbose, "v", "verbosity","0 none, 1 little, 2 debug");
  opt.parse();
  std::cout << "verbose: " << verbose << "\n";    

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "System Test Ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
