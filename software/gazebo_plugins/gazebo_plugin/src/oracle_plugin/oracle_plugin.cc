// minimal two-way LCM gazebo plugin
// - listens to robot pose and sends back the affordance positions
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
class OraclePlugin: public ModelPlugin{
  
   public: 
     OraclePlugin( ):
          est_world_to_headT_(0, Eigen::Isometry3d::Identity()),
          true_world_to_headT_(0, Eigen::Isometry3d::Identity()){ }
  
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  
    void storeAffordances();
  
    drc::affordance_t getAffordance(std::string name, Eigen::Isometry3d pose);
  
    // Called by the world update start event
    void OnUpdate();
    
    ////// All LCM receive thread work:
    void QueueThread(); 
    void on_pose_head(const lcm::ReceiveBuffer* buf, const std::string& channel, const bot_core::pose_t* msg);
  
  private: 
    physics::ModelPtr model; // Pointer to the model
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
    
    std::map< std::string, AffordancePlus > aff_map_;    
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(OraclePlugin)



void OraclePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
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
  this->update_rate_=20; // can increase if needed
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;   
  
  // Name of the model and link we wish the affordances to be relative to:
  this->robot_name_ = "atlas";
  this->world_to_robot_link_ = "pelvis";
  
  model_map_["ground_plane"]=70000;  
  model_map_["standpipe"]=70001; 
  model_map_["sbox1"]=70002;
  model_map_["sbox2"]=70003;  
  model_map_["sbox3"]=70004;    
  model_map_["sbox4"]=70005;      
  model_map_["mit_table"]=70006;
  model_map_["mit_coke_can"]=70007; 
  model_map_["bowl"]=70008;
  model_map_["drc_vehicle"]=70009;
  model_map_["mit_cordless_drill"]=70010;
  model_map_["saucepan"]=70011;  
  model_map_["simple_cylinder"]=70012;  
  //model_map_["mit_drc_robot"]=7012; //dont send this
  
  storeAffordances();
    
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



void OraclePlugin::storeAffordances(){
  int counter=0;
  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.nparams =9;

    a.param_names.push_back("x");
    a.params.push_back(0);
    a.param_names.push_back("y");
    a.params.push_back(0);
    a.param_names.push_back("z");
    a.params.push_back(0);

    a.param_names.push_back("roll");
    a.params.push_back( 0 );
    a.param_names.push_back("pitch");
    a.params.push_back( 0);
    a.param_names.push_back("yaw");
    a.params.push_back( 0 );

    a.param_names.push_back("radius");
    a.params.push_back(0.220000);
    a.param_names.push_back("length");
    a.params.push_back(0.020000);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    
    a.nstates =0;
    a.nptinds =0;
    
    Eigen::Isometry3d offset;
    offset.setIdentity();

    AffordancePlus affp;
    affp.aff =a;
    affp.offset = offset;
    aff_map_["drc_vehicle_steering_wheel"]=affp;
  }
  
  
  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.nparams =9;

    a.param_names.push_back("x");
    a.params.push_back(0);
    a.param_names.push_back("y");
    a.params.push_back(0);
    a.param_names.push_back("z");
    a.params.push_back(0);

    a.param_names.push_back("roll");
    a.params.push_back( 0 );
    a.param_names.push_back("pitch");
    a.params.push_back( 0);
    a.param_names.push_back("yaw");
    a.params.push_back( 0 );

    a.param_names.push_back("radius");
    a.params.push_back(0.020000);
    a.param_names.push_back("length");
    a.params.push_back(0.13);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nstates =0;
    a.nptinds =0;
    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << -0.085, 0.03, 0.20;
    double ypr[3]={0, 1.571,0};
    Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    offset.rotate(quat);

    AffordancePlus affp;
    affp.aff =a;
    affp.offset = offset;
    aff_map_["mit_cordless_drill_link"]=affp;
  }    
  

  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.nparams =9;

    a.param_names.push_back("x");
    a.params.push_back(0);
    a.param_names.push_back("y");
    a.params.push_back(0);
    a.param_names.push_back("z");
    a.params.push_back(0);

    a.param_names.push_back("roll");
    a.params.push_back( 0 );
    a.param_names.push_back("pitch");
    a.params.push_back( 0);
    a.param_names.push_back("yaw");
    a.params.push_back( 0 );

    a.param_names.push_back("radius");
    a.params.push_back(0.030000);
    a.param_names.push_back("length");
    a.params.push_back(0.11);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nstates =0;
    a.nptinds =0;
    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0, -0.025, 0.15;
    //double ypr[3]={0, 0,0};
    //Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    //offset.rotate(quat);

    AffordancePlus affp;
    affp.aff =a;
    affp.offset = offset;
    aff_map_["mit_cordless_drill_link_handle"]=affp;
  }    
    
  
  
  {
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.nparams =9;

    a.param_names.push_back("x");
    a.params.push_back(0);
    a.param_names.push_back("y");
    a.params.push_back(0);
    a.param_names.push_back("z");
    a.params.push_back(0);
    a.param_names.push_back("roll");
    a.params.push_back( 0 );
    a.param_names.push_back("pitch");
    a.params.push_back( 0);
    a.param_names.push_back("yaw");
    a.params.push_back( 0 );
    a.param_names.push_back("radius");
    a.params.push_back(0.10000); // 
    a.param_names.push_back("length");
    a.params.push_back(0.35000); //.32
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nstates =0;
    a.nptinds =0;
    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0.0,0,0.14;
    
    AffordancePlus affp;
    affp.aff =a;
    affp.offset = offset;
    aff_map_["mit_coke_can_link"]=affp;
  }        
  
  

  {
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.nparams =9;

    a.param_names.push_back("x");
    a.params.push_back(0);
    a.param_names.push_back("y");
    a.params.push_back(0);
    a.param_names.push_back("z");
    a.params.push_back(0);
    a.param_names.push_back("roll");
    a.params.push_back( 0 );
    a.param_names.push_back("pitch");
    a.params.push_back( 0);
    a.param_names.push_back("yaw");
    a.params.push_back( 0 );
    a.param_names.push_back("radius");
    a.params.push_back(0.05500); // 
    a.param_names.push_back("length");
    a.params.push_back(0.23000); //.32
    a.param_names.push_back("mass");
    a.params.push_back(0.39); // unknown
    a.nstates =0;
    a.nptinds =0;
    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0.0,0,0.0;
    
    AffordancePlus affp;
    affp.aff =a;
    affp.offset = offset;
    aff_map_["simple_cylinder_link"]=affp;
  }        
    
    
  {
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.nparams =9;

    a.param_names.push_back("x");
    a.params.push_back(0);
    a.param_names.push_back("y");
    a.params.push_back(0);
    a.param_names.push_back("z");
    a.params.push_back(0);
    a.param_names.push_back("roll");
    a.params.push_back( 0 );
    a.param_names.push_back("pitch");
    a.params.push_back( 0);
    a.param_names.push_back("yaw");
    a.params.push_back( 0 );
    a.param_names.push_back("radius");
    a.params.push_back(0.02500); // 
    a.param_names.push_back("length");
    a.params.push_back(0.23000); //.32
    a.param_names.push_back("mass");
    a.params.push_back(0.39); // unknown
    a.nstates =0;
    a.nptinds =0;
    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0.0,0,0.15;
    
    AffordancePlus affp;
    affp.aff =a;
    affp.offset = offset;
    aff_map_["standpipe_standpipe"]=affp;
  }        
        
  
}



drc::affordance_t OraclePlugin::getAffordance(
        std::string name,
        Eigen::Isometry3d pose){

    // Find the affordance:
    AffordancePlus affp = aff_map_.find( name)->second;
    
    drc::affordance_t aff= affp.aff;
    // Update the xyzrpr:
    int ix = std::distance( aff.param_names.begin(), std::find( aff.param_names.begin(), aff.param_names.end(), "x"   ) );
    int iy = std::distance( aff.param_names.begin(), std::find( aff.param_names.begin(), aff.param_names.end(), "y"   ) );
    int iz = std::distance( aff.param_names.begin(), std::find( aff.param_names.begin(), aff.param_names.end(), "z"   ) );
    int iroll = std::distance( aff.param_names.begin(), std::find( aff.param_names.begin(), aff.param_names.end(), "roll"   ) );
    int ipitch = std::distance( aff.param_names.begin(), std::find( aff.param_names.begin(), aff.param_names.end(), "pitch"   ) );
    int iyaw = std::distance( aff.param_names.begin(), std::find( aff.param_names.begin(), aff.param_names.end(), "yaw"   ) );

    pose= pose*affp.offset;
    
    Eigen::Quaterniond r(pose.rotation());
    double yaw, pitch, roll;
    quat_to_euler(r, yaw, pitch, roll);    
    aff.params[ix] = pose.translation().x() ;
    aff.params[iy] = pose.translation().y() ;
    aff.params[iz] = pose.translation().z() ;
    aff.params[iroll] = roll ;
    aff.params[ipitch] = pitch ;
    aff.params[iyaw] = yaw ;
    
    return aff;
}  




void OraclePlugin::OnUpdate(){
  common::Time sim_time = this->world->GetSimTime();
  if (sim_time - this->last_update_time_ >= this->update_period_){

    std::vector<physics::ModelPtr> all_models = this->world->GetModels();
    
    BOOST_FOREACH( physics::ModelPtr model, all_models ){
      if (model){
        if ( model->GetName().compare( this->robot_name_ ) == 0){
          //gzerr << "which link: "<< model->GetName() <<"\n";
          //physics::Link_V all_links = model->GetAllLinks(); // depricated in 1.4.0
          physics::Link_V all_links = model->GetLinks();
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
    
    drc::affordance_collection_t affcol;
    affcol.name;    // name to display e.g. "kitchen" or "pump room"
    affcol.utime=0; // utime of the local map we refer to
    affcol.map_id=0; // id of the local map - duplication of the above?
    affcol.naffs=0;
  
    
    BOOST_FOREACH( physics::ModelPtr model, all_models ){
      if (model){
        //gzerr << "which link: "<< model->GetName() <<"\n";
        map< std::string  ,int>::iterator it;
        it=model_map_.find( model->GetName() );
        if ( it == model_map_.end() ){
          //gzerr << "couldn't find this: "<< model->GetName() <<"\n";
        }else{
          int model_id = model_map_.find(   model->GetName()  )->second;
          // physics::Link_V all_links = model->GetAllLinks(); deprecated in 1.4.0
          physics::Link_V all_links = model->GetLinks();
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
              
              //gzerr << model->GetName() << ", " << link->GetName() << " | "
              //      << pose.pos.x << " " << pose.pos.y << " " << pose.pos.z << " | "
              //      << pose.rot.w << " " << pose.rot.x << " " << pose.rot.y << " " << pose.rot.z << "\n";
              
              // These 3 lines transform the true link position into the estimated robot's frame, i.e:
              // est_w2l =  est_w2h * (true_w2h_inv) * true_w2l
              Eigen::Isometry3d true_robot_to_link;
              true_robot_to_link = true_world_to_headT_.pose.inverse() * world_to_link   ;
              world_to_link = est_world_to_headT_.pose * true_robot_to_link ;
              
              Isometry3dTime world_to_linkT(curr_time+counter, world_to_link);
              world_to_linksT.push_back(world_to_linkT);
              counter++;
              

              std::string affname = model->GetName() + "_" +link->GetName();
              if ( link->GetName().compare( "steering_wheel" ) == 0){
                gzerr<< "got steering_wheel\n"; 
                affcol.affs.push_back ( getAffordance(affname,  world_to_link) );
              }
              if ( link->GetName().compare( "hand_brake" ) == 0){
                gzerr<< "got hand_brake\n"; 
                affcol.affs.push_back ( getAffordance(affname,  world_to_link) );
              }
              if ( model->GetName().compare( "mit_coke_can" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  affcol.affs.push_back ( getAffordance(affname,  world_to_link) );
                }
              }
              if ( model->GetName().compare( "mit_cordless_drill" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  affcol.affs.push_back ( getAffordance(affname,  world_to_link) );
                  affcol.affs.push_back ( getAffordance(  "mit_cordless_drill_link_handle",  world_to_link) );
                }
              }
              if ( model->GetName().compare( "simple_cylinder" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  affcol.affs.push_back ( getAffordance(affname,  world_to_link) );
                }
              }
              if ( model->GetName().compare( "standpipe" ) == 0){
                if ( link->GetName().compare( "standpipe" ) == 0){
                  affcol.affs.push_back ( getAffordance(affname,  world_to_link) );
                }
              }
              
              
            }
          }
          pc_vis_->pose_collection_to_lcm_from_list(model_id, world_to_linksT); // all links in world frame
        }
      }
    }
    
    affcol.naffs = affcol.affs.size();
    lcm_publish_.publish( ("AFFORDANCE_COLLECTION") , &affcol);        
    
    this->last_update_time_ = sim_time;
  }
}


void OraclePlugin::QueueThread(){
    lcm::LCM lcm_subscribe_ ;
    if(!lcm_subscribe_.good()){
      gzerr <<"ERROR: lcm_subscribe_ is not good()\n";
    }
  
    lcm_subscribe_.subscribe("POSE_BODY", &OraclePlugin::on_pose_head, this);
    gzerr << "Launching Oracle LCM handler\n";
    while (0 == lcm_subscribe_.handle());
}    


void OraclePlugin::on_pose_head(const lcm::ReceiveBuffer* buf,
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


}