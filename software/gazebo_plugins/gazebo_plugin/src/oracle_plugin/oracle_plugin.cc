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

struct AffordancePlusMeta
{
  Eigen::Isometry3d offset; // offset between the ros pose and the affordance collection pose
  drc::affordance_plus_t affplus;
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
  
    bool sendAffordance(std::string name, Eigen::Isometry3d pose);
  
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
    std::map< std::string ,int> model_map_;
   
    protected: double update_rate_;
    protected: double update_period_;
    protected: common::Time last_update_time_;    
    
    Isometry3dTime true_world_to_headT_;    
    Isometry3dTime est_world_to_headT_;
    
    std::string world_to_robot_link_;
    std::string robot_name_;
    
    std::map< std::string, AffordancePlusMeta > aff_map_;    
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
  model_map_["duff_beer"]=70012;
  model_map_["steering_assembly"]=70013;    
  model_map_["mit_standpipe"]=70014;    
  model_map_["mit_valve"]=70015;
  model_map_["drill"]=70016;  
  model_map_["table"]=70017;  
  //model_map_["mit_drc_robot"]=7012; //dont send this
  gzerr << "model_map_ " << model_map_.size()<<"\n";
  storeAffordances();
    
  // obj_cfg: id name type reset
  // pts_cfg: id name type reset objcoll usergb rgb
  pc_vis_ = new pointcloud_vis( lcm_publish_.getUnderlyingLCM());
  pc_vis_->obj_cfg_list.push_back( obj_cfg( 70000 , "Oracle",5,1) );
}



void OraclePlugin::storeAffordances()
{

  int counter=0;
  
  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="steering_cyl";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.220000);
    a.param_names.push_back("length");
    a.params.push_back(0.020000);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;
    
    Eigen::Isometry3d offset;
    offset.setIdentity();

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["drc_vehicle_polaris_ranger_ev::steering_wheel"]=affmeta;
  }
  
  
  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.020000);
    a.param_names.push_back("length");
    a.params.push_back(0.13);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << -0.085, 0.03, 0.20;
    double ypr[3]={0, 1.571,0};
    Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["mit_cordless_drill_link"]=affmeta;
  }    
  

  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.030000);
    a.param_names.push_back("length");
    a.params.push_back(0.1);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0, -0.025, 0.125;
    double ypr[3]={1.571, 0,0};
    Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["mit_cordless_drill_link_handle"]=affmeta;
  }    
    
  
  
  {
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.10000); // 
    a.param_names.push_back("length");
    a.params.push_back(0.35000); //.32
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0.0,0,0.14;
    
    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["mit_coke_can_link"]=affmeta;
  }        
  
  

  {
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.05500); // 
    a.param_names.push_back("length");
    a.params.push_back(0.23000); //.32
    a.param_names.push_back("mass");
    a.params.push_back(0.39); // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0.0,0,0.0;
    
    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["duff_beer_link"]=affmeta;
  }        
    
    
  {
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.02500); // 
    a.param_names.push_back("length");
    a.params.push_back(0.23000); //.32
    a.param_names.push_back("mass");
    a.params.push_back(0.39); // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0.0,0,0.15;
    
    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["standpipe_standpipe"]=affmeta;
  }     


  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="steering_cyl";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.220000);
    a.param_names.push_back("length");
    a.params.push_back(0.020000);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    //offset.translation()  << -0.085, 0.03, 0.20;
    double ypr[3]={0,0,-1.571};
    Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["steering_assembly_steering_wheel"]=affmeta;
  }   
        

  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.1000);
    a.param_names.push_back("length");
    a.params.push_back(0.38000);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0,0, 0.30;
    //double ypr[3]={0,0,-1.571};
    //Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    //offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["mit_standpipe_link"]=affmeta;
  }   
  

  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.150000);
    a.param_names.push_back("length");
    a.params.push_back(0.020000);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0, 0, 0.14;
    //double ypr[3]={0,0,-1.571};
    //Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    //offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["mit_valve_wheel"]=affmeta;
  }     
  
  
  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="plane"; // was a box
    a.aff_store_control = drc::affordance_t::NEW;

    /*
    a.param_names.push_back("lX");
    a.params.push_back(3.00000);
    a.param_names.push_back("lY");
    a.params.push_back(3.00000);
    a.param_names.push_back("lZ");
    a.params.push_back(0.01000); 
    a.param_names.push_back("mass");
    a.params.push_back(1.0);  // unknown
    */
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 1.5,0,0;
    double ypr[3]={0, 1.571,0};
    Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["sbox4_chassis"]=affmeta; 
  }     
    
  
  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="plane"; // was a box
    a.aff_store_control = drc::affordance_t::NEW;

    /*
    a.param_names.push_back("lX");
    a.params.push_back(0.8000);
    a.param_names.push_back("lY");
    a.params.push_back(1.5000);
    a.param_names.push_back("lZ");
    a.params.push_back(0.01); 
    a.param_names.push_back("mass");
    a.params.push_back(1.0);  // unknown
    */
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  <<0,0, 1.0;
    double ypr[3]={1.571,0,0};
    Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["mit_table_link"]=affmeta; 
  } 

  
  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="box";// was a box
    a.aff_store_control = drc::affordance_t::NEW;


    a.param_names.push_back("lX");
    a.params.push_back(0.8000);
    a.param_names.push_back("lY");
    a.params.push_back(1.5000);
    a.param_names.push_back("lZ");
    a.params.push_back(0.01); 
    a.param_names.push_back("mass");
    a.params.push_back(1.0);  // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  <<0,0, 1.0;
    double ypr[3]={1.571,0,0};
    Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["table_link"]=affmeta; 
  }

  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;
    a.nparams =9;

    a.param_names.push_back("radius");
    a.params.push_back(0.020000);
    a.param_names.push_back("length");
    a.params.push_back(0.13);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;
    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << -0.085, 0.03, 0.20;
    double ypr[3]={0, 1.571,0};
    Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["drill_link"]=affmeta;
  }      

  { 
    drc::affordance_t a;
    a.utime =0;
    a.map_id =0;
    a.uid =counter++;
    a.otdf_type ="cylinder";
    a.aff_store_control = drc::affordance_t::NEW;

    a.param_names.push_back("radius");
    a.params.push_back(0.030000);
    a.param_names.push_back("length");
    a.params.push_back(0.1);
    a.param_names.push_back("mass");
    a.params.push_back(1.0); // unknown
    a.nparams = a.params.size();
    a.nstates =0;

    
    Eigen::Isometry3d offset;
    offset.setIdentity();
    offset.translation()  << 0, -0.025, 0.125;
    double ypr[3]={1.571, 0,0};
    Eigen::Quaterniond quat = euler_to_quat( ypr[0], ypr[1], ypr[2]);             
    offset.rotate(quat);

    AffordancePlusMeta affmeta;
    drc::affordance_plus_t aplus; aplus.aff = a; aplus.ntriangles=0; aplus.npoints=0; affmeta.affplus = aplus;
    affmeta.offset = offset;
    aff_map_["drill_link_handle"]=affmeta;
  }
    

}



bool OraclePlugin::sendAffordance(
        std::string name,
        Eigen::Isometry3d pose){

    // Find the affordance:
    AffordancePlusMeta affmeta = aff_map_.find( name)->second;

    /*
    cout << aff_map_.find( name)->second.aff.otdf_type << " is the type\n";
    cout << (int) aff_map_.find( name)->second.aff.aff_store_control << " is the control\n";
    cout <<  name << " is combined link\n";
    */
    aff_map_.find( name)->second.affplus.aff.aff_store_control = drc::affordance_t::UPDATE;

    drc::affordance_plus_t affplus= affmeta.affplus;
    // Update the xyzrpr:
    /*
    int ix = std::distance( affplus.aff.param_names.begin(), std::find( affplus.aff.param_names.begin(), affplus.aff.param_names.end(), "x"   ) );
    int iy = std::distance( affplus.aff.param_names.begin(), std::find( affplus.aff.param_names.begin(), affplus.aff.param_names.end(), "y"   ) );
    int iz = std::distance( affplus.aff.param_names.begin(), std::find( affplus.aff.param_names.begin(), affplus.aff.param_names.end(), "z"   ) );
    int iroll = std::distance( affplus.aff.param_names.begin(), std::find( affplus.aff.param_names.begin(), affplus.aff.param_names.end(), "roll"   ) );
    int ipitch = std::distance( affplus.aff.param_names.begin(), std::find( affplus.aff.param_names.begin(), affplus.aff.param_names.end(), "pitch"   ) );
    int iyaw = std::distance( affplus.aff.param_names.begin(), std::find( affplus.aff.param_names.begin(), affplus.aff.param_names.end(), "yaw"   ) );
    */
    pose= pose*affmeta.offset;
    
    Eigen::Quaterniond r(pose.rotation());
    double yaw, pitch, roll;
    quat_to_euler(r, yaw, pitch, roll);   
    affplus.aff.origin_xyz[0] =pose.translation().x() ; 
    affplus.aff.origin_xyz[1] =pose.translation().y() ; 
    affplus.aff.origin_xyz[2] =pose.translation().z() ; 
    affplus.aff.origin_rpy[0] =roll; 
    affplus.aff.origin_rpy[1] =pitch ; 
    affplus.aff.origin_rpy[2] =yaw ; 
 
    /*
    affplus.aff.params[ix] = pose.translation().x() ;
    affplus.aff.params[iy] = pose.translation().y() ;
    affplus.aff.params[iz] = pose.translation().z() ;
    affplus.aff.params[iroll] = roll ;
    affplus.aff.params[ipitch] = pitch ;
    affplus.aff.params[iyaw] = yaw ;
    */
    if (affplus.aff.aff_store_control==0){
      lcm_publish_.publish( ("AFFORDANCE_FIT_ORACLE") , &affplus);        
    }else{
      lcm_publish_.publish( ("AFFORDANCE_TRACK_ORACLE") , &affplus);        
    }
    return true;
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
  

    std::vector<Isometry3dTime> world_to_linksT;
    std::vector< int64_t > world_to_link_utimes;
    std::vector< std::string > link_names;
    int64_t counter =0;
      

    BOOST_FOREACH( physics::ModelPtr model, all_models ){
      if (model){
        std::map< std::string,int>::iterator it;
        it=model_map_.find( model->GetName() );
        if ( it != model_map_.end() ){
          int model_id = model_map_.find(   model->GetName()  )->second;
          // physics::Link_V all_links = model->GetAllLinks(); deprecated in 1.4.0
          physics::Link_V all_links = model->GetLinks();
          //gzerr << "model_id: " << model_id << " with " << all_links.size() << " links\n";

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
               
               //gzerr << model->GetName() << ", " << link->GetName() << "\n";
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
              world_to_link_utimes.push_back( curr_time+counter);
              counter++;
              
              std::string affname = model->GetName() + "_" +link->GetName();
              link_names.push_back( affname  );

              if ( model->GetName().compare("drc_vehicle") == 0) {              
                if ( link->GetName().compare( "polaris_ranger_ev::steering_wheel" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
                //if ( link->GetName().compare( "polaris_ranger_ev::hand_brake" ) == 0){                  
                //  sendAffordance(affname,  world_to_link);
                //}
              }

              if ( model->GetName().compare( "mit_coke_can" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
              }
              if ( model->GetName().compare( "mit_cordless_drill" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  sendAffordance(affname,  world_to_link);
                  sendAffordance(   "mit_cordless_drill_link_handle"   ,  world_to_link);
                }
              }
              if ( model->GetName().compare( "duff_beer" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
              }
              if ( model->GetName().compare( "standpipe" ) == 0){
                if ( link->GetName().compare( "standpipe" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
              }
              if ( model->GetName().compare( "steering_assembly" ) == 0){
                if ( link->GetName().compare( "steering_wheel" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
              }
              if ( model->GetName().compare( "mit_standpipe" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
              }
              if ( model->GetName().compare( "mit_valve" ) == 0){
                if ( link->GetName().compare( "wheel" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
              }
              if ( model->GetName().compare( "sbox4" ) == 0){
                if ( link->GetName().compare( "chassis" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
              }
              if ( model->GetName().compare( "mit_table" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
              }
              if ( model->GetName().compare( "table" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  sendAffordance(affname,  world_to_link);
                }
              }              
              if ( model->GetName().compare( "drill" ) == 0){
                if ( link->GetName().compare( "link" ) == 0){
                  sendAffordance(affname,  world_to_link);
                  sendAffordance(   "drill_link_handle"   ,  world_to_link);
                }
              }
              
            }
          }
        }
      }
    }

    pc_vis_->pose_collection_to_lcm_from_list(70000, world_to_linksT); // all links in world frame
    
    // Link names:
    //pc_vis_->text_collection_to_lcm(70001, 70000, "Oracle [Labels]", link_names, world_to_link_utimes );    

    //affcol.naffs = affcol.affs.size();
    //lcm_publish_.publish( ("AFFORDANCE_ORACLE") , &affcol);        
    
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
