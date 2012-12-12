// minimal two-way LCM gazebo plugin
// - listens to LCM for rotation rate of ROTATING_SCAN
// - publishes the angular position @ 100s of Hz
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

namespace gazebo
{   
class OraclePlugin: public ModelPlugin{    
  
   public: OraclePlugin( ):
   est_world_to_headT_(0, Eigen::Isometry3d::Identity()),
   true_world_to_headT_(0, Eigen::Isometry3d::Identity()){
   }
  
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;
    this->world = _parent->GetWorld();
    
    // LCM receive thread:
    this->callback_queue_thread_ = boost::thread(boost::bind(&OraclePlugin::QueueThread, this));
      
    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&OraclePlugin::OnUpdate, this));

    if(!lcm_publish_.good()){ gzerr <<"ERROR: lcm is not good()" <<std::endl; }

    // Update rate of the publisher in Hz
    this->update_rate_=10; // can increase if needed
    if (this->update_rate_ > 0.0)
      this->update_period_ = 1.0/this->update_rate_;
    else
      this->update_period_ = 0.0;   
    
    // Name of the model and link we wish to move relative to:
    this->robot_name_ = "mit_drc_robot";
    this->world_to_robot_link_ = "head";
    
    model_map_["ground_plane"]=7000;  
    model_map_["standpipe"]=7001; 
    model_map_["sbox1"]=7002;
    model_map_["sbox2"]=7003;  
    model_map_["sbox3"]=7004;    
    model_map_["sbox4"]=7005;      
    model_map_["table"]=7006;
    model_map_["coke_can"]=7007; 
    model_map_["bowl"]=7008;
    model_map_["car"]=7009;
    model_map_["fire_hose"]=7010;
    model_map_["saucepan"]=7011;  
    //model_map_["mit_drc_robot"]=7012; //dont send this
      
    // obj_cfg: id name type reset
    // pts_cfg: id name type reset objcoll usergb rgb
    pc_vis_ = new pointcloud_vis( lcm_publish_.getUnderlyingLCM());
    for( map<std::string, int>::iterator ii=model_map_.begin(); ii!=model_map_.end(); ++ii){
       gzerr << (*ii).first << ": " << (*ii).second << endl;
       stringstream name_out ;
       name_out << "Oracle - " << (*ii).first;
       pc_vis_->obj_cfg_list.push_back( obj_cfg( (*ii).second  ,name_out.str(),5,1) );
    }
  }
    
  // Called by the world update start event
  public: void OnUpdate(){
    common::Time sim_time = this->world->GetSimTime();
    if (sim_time - this->last_update_time_ >= this->update_period_){

      std::list<physics::ModelPtr> all_models = this->world->GetModels();
      
      BOOST_FOREACH( physics::ModelPtr model, all_models ){
        if (model){
          if ( model->GetName().compare( this->robot_name_ ) == 0){
            //gzerr << "which link: "<< model->GetName() <<"\n";
            physics::Link_V all_links = model->GetAllLinks();
            BOOST_FOREACH( physics::LinkPtr link, all_links ){
              if (link){
                if ( link->GetName().compare( this->world_to_robot_link_ ) == 0){
                  //gzerr << "which link: "<< link->GetName() <<"\n";
                  math::Pose pose;
                  pose = link->GetWorldPose();
                  
                  Eigen::Isometry3d true_world_to_head;
                  true_world_to_head.setIdentity();
                  true_world_to_head.translation()  << pose.pos.x, pose.pos.y, pose.pos.z;
                  Eigen::Quaterniond quat = Eigen::Quaterniond( pose.rot.w, pose.rot.x , pose.rot.y , pose.rot.z);
                  true_world_to_head.rotate(quat);    
                  
                  true_world_to_headT_.utime = 0;
                  true_world_to_headT_.pose = true_world_to_head;
                  
                }
              }
            }
            // found robot's head, leave this
            break;
          }
        }
      }
      
      int64_t curr_time = (int64_t) round(sim_time.Double()*1E6);
      BOOST_FOREACH( physics::ModelPtr model, all_models ){
        if (model){
          //gzerr << "which link: "<< model->GetName() <<"\n";
          map< std::string  ,int>::iterator it;
          it=model_map_.find( model->GetName() );
          if ( it == model_map_.end() ){
            //gzerr << "couldn't find this: "<< model->GetName() <<"\n";
          }else{
            int model_id = model_map_.find(   model->GetName()  )->second;
            physics::Link_V all_links = model->GetAllLinks();
            //gzerr << "model_id: " << model_id << " with " << all_links.size() << " links\n";

            int64_t counter =0;
            std::vector<Isometry3dTime> world_to_linksT;
            BOOST_FOREACH( physics::LinkPtr link, all_links ){
              if (link){
                //gzerr << "which link: "<< link->GetName() <<"\n";
                math::Pose pose;
                pose = link->GetWorldPose();
                Eigen::Isometry3d world_to_link;
                world_to_link.setIdentity();
                world_to_link.translation()  << pose.pos.x, pose.pos.y, pose.pos.z;
                Eigen::Quaterniond quat = Eigen::Quaterniond( pose.rot.w, pose.rot.x , pose.rot.y , pose.rot.z);
                world_to_link.rotate(quat);    
                
                // These 3 lines transform the true link position into the estimated robot's frame, i.e:
                // est_w2l =  est_w2h * (true_w2h_inv) * true_w2l
                Eigen::Isometry3d true_robot_to_link;
                true_robot_to_link = true_world_to_headT_.pose.inverse() * world_to_link   ;
                world_to_link = est_world_to_headT_.pose * true_robot_to_link ;
                
                Isometry3dTime world_to_linkT(curr_time+counter, world_to_link);
                world_to_linksT.push_back(world_to_linkT);
                counter++;
              }
            }
            pc_vis_->pose_collection_to_lcm_from_list(model_id, world_to_linksT); // all links in world frame
          }
        }
        
      }
      this->last_update_time_ = sim_time;
    }
  }

  ////// All LCM receive thread work:
  public: void QueueThread(){
    lcm::LCM lcm_subscribe_ ;
    if(!lcm_subscribe_.good()){
      gzerr <<"ERROR: lcm_subscribe_ is not good()\n";
    }
  
    lcm_subscribe_.subscribe("POSE_HEAD", &OraclePlugin::on_pose_head, this);
    gzerr << "Launching Oracle LCM handler\n";
    while (0 == lcm_subscribe_.handle());
  }    
    
  public: void on_pose_head(const lcm::ReceiveBuffer* buf,
                    const std::string& channel,
                    const bot_core::pose_t* msg){
    // Store the new pose estimate:
    //gzerr << msg->utime << " is new pose time\n";
    Eigen::Isometry3d world_to_head;
    world_to_head.setIdentity();
    world_to_head.translation()  << msg->pos[0], msg->pos[1], msg->pos[2];
    Eigen::Quaterniond quat = Eigen::Quaterniond( msg->orientation[0], msg->orientation[1],
                                                  msg->orientation[2], msg->orientation[3]);
    world_to_head.rotate(quat);    
    est_world_to_headT_.utime = msg->utime;
    est_world_to_headT_.pose = world_to_head;
  }  
  
  // Pointer to the model
  private: 
    physics::ModelPtr model;
    physics::WorldPtr world;
    lcm::LCM lcm_publish_ ;
    double target_velocity;
    boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    pointcloud_vis* pc_vis_;
    map< std::string ,int> model_map_;
    
    protected: double update_rate_;
    protected: double update_period_;
    protected: common::Time last_update_time_;    
    
    Isometry3dTime true_world_to_headT_;    
    Isometry3dTime est_world_to_headT_;
    
    std::string world_to_robot_link_;
    std::string robot_name_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(OraclePlugin)
}

