// minimal two-way LCM gazebo plugin
// - rateset
// mfallon nov 2012
#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <pointcloud_tools/pointcloud_math.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

struct AffordancePlus
{
  Eigen::Isometry3d offset; // offset between the ros pose and the affordance collection pose
  drc::affordance_t aff;
};


namespace gazebo
{
class RateSetPlugin: public ModelPlugin{
  
   public: 
     RateSetPlugin( ){ }
  
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  
    // Called by the world update start event
    void OnUpdate();
    
    ////// All LCM receive thread work:
    void QueueThread(); 
    void on_rate_set(const lcm::ReceiveBuffer* buf, const std::string& channel, const bot_core::pose_t* msg);
  
    void on_pose_set(const lcm::ReceiveBuffer* buf, const std::string& channel, const bot_core::pose_t* msg);

  private: 
    physics::ModelPtr model; // Pointer to the model
    physics::WorldPtr world;
    lcm::LCM lcm_publish_ ;
    boost::thread callback_queue_thread_;

    double fraction_sleep_;    

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
   
    protected: double update_rate_;
    protected: double update_period_;
    protected: common::Time last_update_time_;    
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RateSetPlugin)



void RateSetPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  // Store the pointer to the model
  this->model = _parent;
  this->world = _parent->GetWorld();
  
  // LCM receive thread:
  this->callback_queue_thread_ = boost::thread(boost::bind(&RateSetPlugin::QueueThread, this));
    
  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
        boost::bind(&RateSetPlugin::OnUpdate, this));

  if(!lcm_publish_.good()){ gzerr <<"ERROR: lcm is not good()" <<std::endl; }

  // Update rate of the publisher in Hz
  this->update_rate_=1000; // can increase if needed
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;   
  
  // Name of the model and link we wish the affordances to be relative to:
  //this->robot_name_ = "atlas";
  //this->world_to_robot_link_ = "pelvis";

  fraction_sleep_ = 0.0;
}


void RateSetPlugin::OnUpdate(){
  common::Time sim_time = this->world->GetSimTime();
  if (sim_time - this->last_update_time_ >= this->update_period_){
    //gzerr << "start sleep "<< sim_time << " ["<< fraction_sleep_ <<"]\n";
    usleep( 1000*1000*fraction_sleep_);
    //gzerr << "end sleep " << sim_time <<"\n"; 
    this->last_update_time_ = sim_time;
  }
}


void RateSetPlugin::QueueThread(){
    lcm::LCM lcm_subscribe_ ;
    if(!lcm_subscribe_.good()){
      gzerr <<"ERROR: lcm_subscribe_ is not good()\n";
    }
  
    lcm_subscribe_.subscribe("GAZEBO_RATESET", &RateSetPlugin::on_rate_set, this);
    lcm_subscribe_.subscribe("POSE_SET", &RateSetPlugin::on_pose_set, this);

    gzerr << "Launching RateSet LCM handler\n";
    while (0 == lcm_subscribe_.handle());
}    

void RateSetPlugin::on_pose_set(const lcm::ReceiveBuffer* buf,
                    const std::string& channel,
                    const bot_core::pose_t* msg){

    Eigen::Isometry3d link_to_hose;
    link_to_hose.setIdentity();
    std::string rel_model = "atlas";
    std::string rel_link = "right_f0_0";
    if (msg->pos[0]==0){
      link_to_hose.translation()  << -0.025,0.02,0.072;
      double ypr[3]={1.571, 0,0};
      Eigen::Quaterniond quat2 = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
      link_to_hose.rotate(quat2);
      gzerr << "will set AT HAND\n";
    }else if(msg->pos[0] == 1){
      link_to_hose.translation()  << -0.025,0.02,0.102;
      double ypr[3]={1.571, 0,0};
      Eigen::Quaterniond quat2 = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
      link_to_hose.rotate(quat2);
      gzerr << "will set near HAND\n";
    }else if(msg->pos[0] == 2){
      link_to_hose.translation()  << -0.025,0.02,0.272;
      double ypr[3]={1.571, 0,0};
      Eigen::Quaterniond quat2 = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
      link_to_hose.rotate(quat2);
      gzerr << "will set OFF HAND\n";
    }else if(msg->pos[0] == 3){ 
      rel_model = "standpipe";
      rel_link = "standpipe";
      link_to_hose.translation()  << 0.0,0.0,0.023;
      double ypr[3]={1.2, 1.571,0};  //  ??  XXX  ??? 
      // 0.001784 -4.6e-05 0.023 1.56985 1.55991 -0.000936
      Eigen::Quaterniond quat2 = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
      link_to_hose.rotate(quat2);
      gzerr << "will set AT PIPE\n";
    }else if(msg->pos[0] == 4){ 
      rel_model = "standpipe";
      rel_link = "standpipe";
      link_to_hose.translation()  << 0.0,0.0,-0.135;
      double ypr[3]={1.571, 1.571,0};
      // 0.001784 -4.6e-05 0.023 1.56985 1.55991 -0.000936
      Eigen::Quaterniond quat2 = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
      link_to_hose.rotate(quat2);
      gzerr << "will set OFF PIPE\n";
    }else{
      gzerr << "Command not understood\n";
      return;	
    }

    std::vector<physics::ModelPtr> all_models = this->world->GetModels();

    math::Pose pose_hand;
    BOOST_FOREACH( physics::ModelPtr model, all_models ){
      if (model){
        //gzerr << "which model: "<< model->GetName() <<"\n";
        if ( model->GetName() == rel_model ){
          //gzerr << "got: "<< model->GetName() <<"\n"; 
          physics::Link_V all_links = model->GetLinks();
          BOOST_FOREACH( physics::LinkPtr link, all_links ){
            //gzerr << "which link: "<< link->GetName() <<"\n";
            if (link->GetName() == rel_link ){
              //gzerr << "which link: "<< link->GetName() <<"\n";
              
              pose_hand=link->GetWorldPose();
              //gzerr << model->GetName() << ", " << link->GetName() << "\n";
              //gzerr << model->GetName() << ", " << link->GetName() << " | "
              //      << pose_hand.pos.x << " " << pose_hand.pos.y << " " << pose_hand.pos.z << " | "
              //      << pose_hand.rot.w << " " << pose_hand.rot.x << " " << pose_hand.rot.y << " " << pose_hand.rot.z << " hand\n";
            }
          }
        }
      }
    }

    BOOST_FOREACH( physics::ModelPtr model, all_models ){
      if (model){
        if ( model->GetName() == "vrc_firehose_long" ){
          //gzerr << "got: "<< model->GetName() <<"\n"; 
          physics::Link_V all_links = model->GetLinks();
          BOOST_FOREACH( physics::LinkPtr link, all_links ){
            if (link->GetName() == "coupling"){
              //gzerr << "which link: "<< link->GetName() <<"\n";
              
              
              Eigen::Isometry3d world_to_link;
              world_to_link.setIdentity();
              world_to_link.translation()  << pose_hand.pos.x, pose_hand.pos.y, pose_hand.pos.z;
              Eigen::Quaterniond quat = Eigen::Quaterniond( pose_hand.rot.w, pose_hand.rot.x , pose_hand.rot.y , pose_hand.rot.z);
              world_to_link.rotate(quat);   

              Eigen::Isometry3d world_to_hose = world_to_link*link_to_hose;
 
              math::Pose pose;
              pose.pos.x =world_to_hose.translation().x() ; 
              pose.pos.y =world_to_hose.translation().y() ; 
              pose.pos.z =world_to_hose.translation().z() ; 
              Eigen::Quaterniond r(world_to_hose.rotation());
              pose.rot.w =r.w() ;
              pose.rot.x =r.x() ;
              pose.rot.y =r.y() ;
              pose.rot.z =r.z() ;

              link->SetWorldPose(pose);
              //gzerr << model->GetName() << ", " << link->GetName() << " | "
              //      << pose_hand.pos.x << " " << pose_hand.pos.y << " " << pose_hand.pos.z << " | "
              //      << pose_hand.rot.w << " " << pose_hand.rot.x << " " << pose_hand.rot.y << " " << pose_hand.rot.z << " after\n";

              link->SetLinearVel( gazebo::math::Vector3(0, 0, 0) );
              link->SetAngularVel( gazebo::math::Vector3(0, 0, 0) );
            }
          }
        }
      }
    }
}


void RateSetPlugin::on_rate_set(const lcm::ReceiveBuffer* buf,
                    const std::string& channel,
                    const bot_core::pose_t* msg){
    // Store the new pose estimate:
    //gzerr << msg->utime << " is new rateset time\n";
    gzerr << "New rate. Will now aim for "<< 
        1/( msg->pos[0] +1)  << " times realtime\n";
    fraction_sleep_ = msg->pos[0]/this->update_rate_;

} 


}
