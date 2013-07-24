
#include "AffordanceCollectionManager.hpp"


using namespace visualization_utils;
using namespace collision;

//==================constructor / destructor
AffordanceCollectionManager::AffordanceCollectionManager(boost::shared_ptr<lcm::LCM> &lcm):_lcm(lcm)
{
  std::string otdf_dir_name = string(getModelsPath()) + "/otdf/"; // getModelsPath gives /drc/software/build/models/
  cout << "searching for otdf files in: "<< otdf_dir_name << endl;
  get_OTDF_filenames_from_dir(otdf_dir_name.c_str(),_otdf_filenames);
 _objects.clear();
 _instance_cnt.clear();
  for(size_t i=0;i<_otdf_filenames.size();i++)
  {
    _instance_cnt.insert(make_pair(_otdf_filenames[i], (int)0));
  }
 
}
//-------------------------------------------------------------------------------------------  
AffordanceCollectionManager::~AffordanceCollectionManager() {

}
//-------------------------------------------------------------------------------------------  
// creates new and sends a msg to store
void AffordanceCollectionManager::create(std::string &filename, bool local_testing)
{
    std::string xml_string;
    if(!otdf::get_xml_string_from_file(filename, xml_string)){
     return; // file extraction failed
    }
    OtdfInstanceStruc instance_struc;
    instance_struc._otdf_instance = otdf::parseOTDF(xml_string);
    if (!instance_struc._otdf_instance){
      std::cerr << "ERROR: Model Parsing of " << filename << " the xml failed" << std::endl;
    }
    // TODO: create a KDL tree parser from OTDF instance, without having to convert to urdf.
    // otdf can contain some elements that are not part of urdf. e.g. TORUS  
    std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(instance_struc._otdf_instance);
    
    std::map<std::string, int >::iterator it;
    it= _instance_cnt.find(filename);
    it->second = it->second + 1;
    std::stringstream oss;
    oss << filename << "_"<< it->second-1;  // unique name, cnt starts from zero
    
    instance_struc.otdf_type=filename;//new string(filename);
    instance_struc.uid=it->second-1; // starts from zero
    //instance_struc._otdf_instance->name_ = oss.str();
    
    instance_struc._collision_detector.reset();
     // Each object has its own collision detector for now.      
    instance_struc._collision_detector = boost::shared_ptr<Collision_Detector>(new Collision_Detector());
    instance_struc._gl_object = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(instance_struc._otdf_instance,instance_struc._collision_detector,true,oss.str()));
    instance_struc._gl_object->set_state(instance_struc._otdf_instance);

    if(local_testing)
      _objects.insert(std::make_pair(oss.str(), instance_struc));
    else
    {
      publish_new_otdf_instance_to_affstore("AFFORDANCE_FIT",(instance_struc.otdf_type),0,instance_struc._otdf_instance); 
    }
    //bot_viewer_request_redraw(self->viewer); // TODO: move to client
}
//-------------------------------------------------------------------------------------------  
// adds to cache given a affordance msg from store
bool AffordanceCollectionManager::add(std::string &filename, const drc::affordance_t &aff, OtdfInstanceStruc &instance_struc)
//OtdfInstanceStruc AffordanceCollectionManager::add(std::string &filename, const drc::affordance_t &aff)
{
 
  std::string xml_string;
  if(!otdf::get_xml_string_from_file(filename, xml_string)){
    return false; // file extraction failed
  }
  
  instance_struc._otdf_instance = otdf::parseOTDF(xml_string);
  if (!instance_struc._otdf_instance){
    std::cerr << "ERROR: Model Parsing of " << filename << " the xml failed" << std::endl;
  }
  
  //instance_struc._otdf_instance->name_ = aff.name;

  // set non-standard params from affordance message
  for (size_t i=0; i < (size_t)aff.nparams; i++)
    {   
      instance_struc._otdf_instance->setParam(aff.param_names[i],aff.params[i]);   
    }

  
  // set standard params from affordance message
  instance_struc._otdf_instance->setParam("x",aff.origin_xyz[0]);
  instance_struc._otdf_instance->setParam("y",aff.origin_xyz[1]);
  instance_struc._otdf_instance->setParam("z",aff.origin_xyz[2]);
  instance_struc._otdf_instance->setParam("roll", aff.origin_rpy[0]);
  instance_struc._otdf_instance->setParam("pitch",aff.origin_rpy[1]);
  instance_struc._otdf_instance->setParam("yaw",  aff.origin_rpy[2]);

  instance_struc._otdf_instance->update();
  
    //set All JointStates too.
  // set non-standard params from affordance message
   for (size_t i=0; i < (size_t)aff.nstates; i++)
   {   
       instance_struc._otdf_instance->setJointState(aff.state_names[i],aff.states[i],0);   
   }

  
  // create a KDL tree parser from OTDF instance, without having to convert to urdf.
  // otdf can contain some elements that are not part of urdf. e.g. TORUS, DYNAMIC_MESH (They are handled as special cases)  
  std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(instance_struc._otdf_instance);
  
  /*std::map<std::string, int >::iterator it;
  it= _parent_affordance_renderer->instance_cnt.find(filename);
  it->second = it->second + 1;*/
  std::stringstream oss;
  oss << aff.otdf_type << "_"<< aff.uid;
  instance_struc.uid = aff.uid;
  instance_struc.otdf_type = aff.otdf_type;//new string(aff.otdf_type);
  instance_struc._collision_detector.reset();
  instance_struc._collision_detector = boost::shared_ptr<Collision_Detector>(new Collision_Detector());
  instance_struc._gl_object = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(instance_struc._otdf_instance,instance_struc._collision_detector,true,oss.str()));
  instance_struc._gl_object->set_state(instance_struc._otdf_instance);

  _objects.insert(std::make_pair(oss.str(), instance_struc));

  return true;
} 

//-------------------------------------------------------------------------------------------  
void AffordanceCollectionManager::update(const drc::affordance_t &aff)
{

  std::stringstream oss;
  oss << aff.otdf_type << "_"<< aff.uid; 

  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
  object_instance_map_type_::iterator it = _objects.find(oss.str());

  // set non-standard params from affordance message
   for (size_t i=0; i < (size_t)aff.nparams; i++)
   {   
       it->second._otdf_instance->setParam(aff.param_names[i],aff.params[i]);   
   }

  // set standard params from affordance message
  it->second._otdf_instance->setParam("x",aff.origin_xyz[0]);
  it->second._otdf_instance->setParam("y",aff.origin_xyz[1]);
  it->second._otdf_instance->setParam("z",aff.origin_xyz[2]);
  it->second._otdf_instance->setParam("roll", aff.origin_rpy[0]);
  it->second._otdf_instance->setParam("pitch",aff.origin_rpy[1]);
  it->second._otdf_instance->setParam("yaw",  aff.origin_rpy[2]);
       
  //set All JointStates too.
  // set non-standard params from affordance message
   for (size_t i=0; i < (size_t)aff.nstates; i++)
   {   
       it->second._otdf_instance->setJointState(aff.state_names[i],aff.states[i],0);   
   }

    it->second._otdf_instance->update();
    if(it->second.otdf_instance_viz_object_sync)
      it->second._gl_object->set_state(it->second._otdf_instance);  
 }
//-------------------------------------------------------------------------------------------  
  
void AffordanceCollectionManager::delete_otdf_from_affstore(string channel, string otdf_type, int uid)
  {
        
        // create and send delete affordance message
        drc::affordance_plus_t aff;
        aff.aff.aff_store_control = drc::affordance_t::DELETE;
        aff.aff.otdf_type = otdf_type;
        aff.aff.uid = uid;
        aff.aff.map_id = 0;
        aff.aff.nparams = 0;
        aff.aff.nstates = 0;
        aff.npoints = 0;
        aff.ntriangles = 0;
        _lcm->publish(channel, &aff);
        cout << "Delete message sent for: " << otdf_type << "_" << uid << endl;
  }


// affstore sync methods 
// TODO: doesn't make sense to pass instance as a parameter
//-------------------------------------------------------------------------------------------  
  void AffordanceCollectionManager::update_mate_joints_in_affstore(string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in, KDL::Frame &T_mate_start_mate_end)
  {
 
   drc::affordance_t msg;

   // get otdf from map
   stringstream nameSS;
   OtdfInstanceStruc* otdf = NULL;
   nameSS << otdf_type << "_" << uid;
   if(_objects.count(nameSS.str())){
     otdf = &_objects[nameSS.str()];
   } else{
     cout << "***** ERROR: " << nameSS.str() << " not found in instantiated_objects\n";
   }

   msg.utime = 0;
   msg.map_id = 0;
   
   
   msg.otdf_type = otdf_type;
   msg.aff_store_control = msg.UPDATE;//msg.NEW
   msg.uid = uid; 
   
    //map<string, double >::iterator obj_it = instance_in->params_map_.find("x");
   msg.origin_xyz[0] =instance_in->params_map_.find("x")->second;
   msg.origin_xyz[1] =instance_in->params_map_.find("y")->second;
   msg.origin_xyz[2] =instance_in->params_map_.find("z")->second;
   msg.origin_rpy[0] =instance_in->params_map_.find("roll")->second;
   msg.origin_rpy[1] =instance_in->params_map_.find("pitch")->second;
   msg.origin_rpy[2] =instance_in->params_map_.find("yaw")->second;

   double bounding_xyz[]={0,0,0};
   double bounding_rpy[]={0,0,0};
   double bounding_lwh[]={0,0,0};
  
   if(otdf){
      bounding_xyz[0] = otdf->boundingBoxXYZ[0];
      bounding_xyz[1] = otdf->boundingBoxXYZ[1];
      bounding_xyz[2] = otdf->boundingBoxXYZ[2];
      bounding_rpy[0] = otdf->boundingBoxRPY[0];
      bounding_rpy[1] = otdf->boundingBoxRPY[1];
      bounding_rpy[2] = otdf->boundingBoxRPY[2];
      bounding_lwh[0] = otdf->boundingBoxLWH[0];
      bounding_lwh[1] = otdf->boundingBoxLWH[1];
      bounding_lwh[2] = otdf->boundingBoxLWH[2];
      msg.modelfile = otdf->modelfile;
   }

   // if common shape (e.g. cylinder, sphere), fill in bounding box
   commonShapeBoundingBox(otdf_type, instance_in, bounding_xyz, bounding_rpy, bounding_lwh);

   msg.bounding_xyz[0] = bounding_xyz[0]; msg.bounding_xyz[1] = bounding_xyz[1]; msg.bounding_xyz[2] = bounding_xyz[2];
   msg.bounding_rpy[0] = bounding_rpy[0]; msg.bounding_rpy[1] = bounding_rpy[1];msg.bounding_rpy[2] = bounding_rpy[2];
   msg.bounding_lwh[0] = bounding_lwh[0]; msg.bounding_lwh[1] = bounding_lwh[1];msg.bounding_lwh[2] = bounding_lwh[2];
    
   typedef std::map<std::string, double > params_mapType;
   for( params_mapType::const_iterator it = instance_in->params_map_.begin(); it!=instance_in->params_map_.end(); it++)
   { 
      // don't copy xyz ypr
      if(isRedundantParam(it->first)) continue;
      // copy all other params
      msg.param_names.push_back(it->first);
      msg.params.push_back(it->second);
   }
   msg.nparams =  msg.param_names.size();

  int cnt=0;
   typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator it = instance_in->joints_.begin();it != instance_in->joints_.end(); it++)
    {     
      if(it->second->type!=(int) otdf::Joint::FIXED) {
        string token  = "mate::";
        string joint_name = it->first;
        double pos, vel;
        size_t found = joint_name.find(token);  
        if (found!=std::string::npos) 
        {
          vel=0;
          double r,p,y;

          //Mate Joints are Intrinsic XYZ (roll::pitch::yaw)
          //Rx(1)*Ry(2)*Rz(3)	
          //c2c3	c2s3	-s2
          //s1s2c3-c1s3	s1s2s3+c1c3	s1c2
          //c1s2c3+s1s3	c1s2s3-s1c3	c1c2

          /*double qx,qy,qz,qw,roll_a,roll_b,pitch_sin,yaw_a,yaw_b;
          T_mate_start_mate_end.M.GetQuaternion(qx,qy,qz,qw);
          roll_a = 2 * (qy*qz - qw*qx);
          roll_b = 1 - 2 * (qx*qx + qy*qy);
          r = atan2 (-roll_a, roll_b);
          pitch_sin = 2 * (qx*qz + qw*qy);
          p = asin (pitch_sin);//
          yaw_a = 2 * (qx*qy - qw*qz);
          yaw_b = 1 - 2 * (qy*qy + qz*qz);
          y = atan2 (-yaw_a, yaw_b);   */
          
          double R12,R11,R13,R23,R33;
          R11 = T_mate_start_mate_end.M.data[0];
          R12 = T_mate_start_mate_end.M.data[1];
          R13 = T_mate_start_mate_end.M.data[2];
          R23 = T_mate_start_mate_end.M.data[5];
          R33 = T_mate_start_mate_end.M.data[8];
          r = atan2 (-R23, R33);
          p = asin (R13);
          y = atan2 (-R12,R11);
                     
          if(joint_name=="mate::x")
            pos = T_mate_start_mate_end.p[0];
          else if(joint_name=="mate::y")
            pos = T_mate_start_mate_end.p[1];          
          else if(joint_name=="mate::z")
            pos = T_mate_start_mate_end.p[2];
          else if(joint_name=="mate::roll")
            pos = r;
          else if(joint_name=="mate::pitch")
            pos = p;     
          else if(joint_name=="mate::yaw")
            pos = y;    
        }
        else
        {
          instance_in->getJointState(it->first,pos,vel);
        }
        cnt++;
        msg.state_names.push_back(it->first);
        msg.states.push_back(pos); 
      }
      
     }
   msg.nstates =  cnt;
   //cout <<"publish_otdf_instance_to_affstore: "<< msg.otdf_type << "_"<< msg.uid << ", of template :" << msg.otdf_type << endl;
   _lcm->publish(channel, &msg);
  }
  
  
//-------------------------------------------------------------------------------------------  
  void AffordanceCollectionManager::update_object_pose_in_affstore(string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in, KDL::Frame &T_world_object)
  {

   drc::affordance_t msg;

   // get otdf from map
   stringstream nameSS;
   OtdfInstanceStruc* otdf = NULL;
   nameSS << otdf_type << "_" << uid;
   if(_objects.count(nameSS.str())){
     otdf = &_objects[nameSS.str()];
   } else{
     cout << "***** ERROR: " << nameSS.str() << " not found in instantiated_objects\n";
   }

   msg.utime = 0;
   msg.map_id = 0;
   
   
   msg.otdf_type = otdf_type;
   msg.aff_store_control = msg.UPDATE;//msg.NEW
   msg.uid = uid; 
   
    //map<string, double >::iterator obj_it = instance_in->params_map_.find("x");
   msg.origin_xyz[0] =T_world_object.p[0];
   msg.origin_xyz[1] =T_world_object.p[1];
   msg.origin_xyz[2] =T_world_object.p[2];
   double r,p,y;
   T_world_object.M.GetRPY(r,p,y);
   msg.origin_rpy[0] =r;
   msg.origin_rpy[1] =p;
   msg.origin_rpy[2] =y;
   

   double bounding_xyz[]={0,0,0};
   double bounding_rpy[]={0,0,0};
   double bounding_lwh[]={0,0,0};
  
   if(otdf){
      bounding_xyz[0] = otdf->boundingBoxXYZ[0];
      bounding_xyz[1] = otdf->boundingBoxXYZ[1];
      bounding_xyz[2] = otdf->boundingBoxXYZ[2];
      bounding_rpy[0] = otdf->boundingBoxRPY[0];
      bounding_rpy[1] = otdf->boundingBoxRPY[1];
      bounding_rpy[2] = otdf->boundingBoxRPY[2];
      bounding_lwh[0] = otdf->boundingBoxLWH[0];
      bounding_lwh[1] = otdf->boundingBoxLWH[1];
      bounding_lwh[2] = otdf->boundingBoxLWH[2];
      msg.modelfile = otdf->modelfile;
   }

   // if common shape (e.g. cylinder, sphere), fill in bounding box
   commonShapeBoundingBox(otdf_type, instance_in, bounding_xyz, bounding_rpy, bounding_lwh);

   msg.bounding_xyz[0] = bounding_xyz[0]; msg.bounding_xyz[1] = bounding_xyz[1]; msg.bounding_xyz[2] = bounding_xyz[2];
   msg.bounding_rpy[0] = bounding_rpy[0]; msg.bounding_rpy[1] = bounding_rpy[1];msg.bounding_rpy[2] = bounding_rpy[2];
   msg.bounding_lwh[0] = bounding_lwh[0]; msg.bounding_lwh[1] = bounding_lwh[1];msg.bounding_lwh[2] = bounding_lwh[2];
    
   typedef std::map<std::string, double > params_mapType;
   for( params_mapType::const_iterator it = instance_in->params_map_.begin(); it!=instance_in->params_map_.end(); it++)
   { 
      // don't copy xyz ypr
      if(isRedundantParam(it->first)) continue;
      // copy all other params
      msg.param_names.push_back(it->first);
      msg.params.push_back(it->second);
   }
   msg.nparams =  msg.param_names.size();

  int cnt=0;
   typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator it = instance_in->joints_.begin();it != instance_in->joints_.end(); it++)
    {     
      if(it->second->type!=(int) otdf::Joint::FIXED) {

          double pos, vel;
          instance_in->getJointState(it->first,pos,vel);
          cnt++;
          msg.state_names.push_back(it->first);
          msg.states.push_back(pos);
      }
     }
   msg.nstates =  cnt;
   //cout <<"publish_otdf_instance_to_affstore: "<< msg.otdf_type << "_"<< msg.uid << ", of template :" << msg.otdf_type << endl;
   _lcm->publish(channel, &msg);
  }
  
//-------------------------------------------------------------------------------------------  
  void AffordanceCollectionManager::publish_otdf_instance_to_affstore(string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in)
  {
   drc::affordance_t msg;

   // get otdf from map
   stringstream nameSS;
   OtdfInstanceStruc* otdf = NULL;
   nameSS << otdf_type << "_" << uid;
   if(_objects.count(nameSS.str())){
     otdf = &_objects[nameSS.str()];
   } else{
     cout << "***** ERROR: " << nameSS.str() << " not found in instantiated_objects\n";
   }

   msg.utime = 0;
   msg.map_id = 0;
   
   
   msg.otdf_type = otdf_type;
   msg.aff_store_control = msg.UPDATE;//msg.NEW
   msg.uid = uid; 
   
    //map<string, double >::iterator obj_it = instance_in->params_map_.find("x");
   msg.origin_xyz[0] =instance_in->params_map_.find("x")->second;
   msg.origin_xyz[1] =instance_in->params_map_.find("y")->second;
   msg.origin_xyz[2] =instance_in->params_map_.find("z")->second;
   msg.origin_rpy[0] =instance_in->params_map_.find("roll")->second;
   msg.origin_rpy[1] =instance_in->params_map_.find("pitch")->second;
   msg.origin_rpy[2] =instance_in->params_map_.find("yaw")->second;
   

   double bounding_xyz[]={0,0,0};
   double bounding_rpy[]={0,0,0};
   double bounding_lwh[]={0,0,0};
  
   if(otdf){
      bounding_xyz[0] = otdf->boundingBoxXYZ[0];
      bounding_xyz[1] = otdf->boundingBoxXYZ[1];
      bounding_xyz[2] = otdf->boundingBoxXYZ[2];
      bounding_rpy[0] = otdf->boundingBoxRPY[0];
      bounding_rpy[1] = otdf->boundingBoxRPY[1];
      bounding_rpy[2] = otdf->boundingBoxRPY[2];
      bounding_lwh[0] = otdf->boundingBoxLWH[0];
      bounding_lwh[1] = otdf->boundingBoxLWH[1];
      bounding_lwh[2] = otdf->boundingBoxLWH[2];
      msg.modelfile = otdf->modelfile;
   }

   // if common shape (e.g. cylinder, sphere), fill in bounding box
   commonShapeBoundingBox(otdf_type, instance_in, bounding_xyz, bounding_rpy, bounding_lwh);

   msg.bounding_xyz[0] = bounding_xyz[0]; msg.bounding_xyz[1] = bounding_xyz[1]; msg.bounding_xyz[2] = bounding_xyz[2];
   msg.bounding_rpy[0] = bounding_rpy[0]; msg.bounding_rpy[1] = bounding_rpy[1];msg.bounding_rpy[2] = bounding_rpy[2];
   msg.bounding_lwh[0] = bounding_lwh[0]; msg.bounding_lwh[1] = bounding_lwh[1];msg.bounding_lwh[2] = bounding_lwh[2];
    
   typedef std::map<std::string, double > params_mapType;
   for( params_mapType::const_iterator it = instance_in->params_map_.begin(); it!=instance_in->params_map_.end(); it++)
   { 
      // don't copy xyz ypr
      if(isRedundantParam(it->first)) continue;
      // copy all other params
      msg.param_names.push_back(it->first);
      msg.params.push_back(it->second);
   }
   msg.nparams =  msg.param_names.size();

  int cnt=0;
   typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator it = instance_in->joints_.begin();it != instance_in->joints_.end(); it++)
    {     
      if(it->second->type!=(int) otdf::Joint::FIXED) {

          double pos, vel;
          instance_in->getJointState(it->first,pos,vel);
          cnt++;
          msg.state_names.push_back(it->first);
          msg.states.push_back(pos);
      }
     }
   msg.nstates =  cnt;
   cout <<"publish_otdf_instance_to_affstore: "<< msg.otdf_type << "_"<< msg.uid << ", of template :" << msg.otdf_type << endl;
   
   _lcm->publish(channel, &msg);

  } 
//-------------------------------------------------------------------------------------------  
  void AffordanceCollectionManager::publish_new_otdf_instance_to_affstore( string channel, string otdf_type, int uid, const boost::shared_ptr<otdf::ModelInterface> instance_in)
  {
   drc::affordance_plus_t msg;

   msg.aff.utime = 0;
   msg.aff.map_id = 0;
   msg.aff.uid =0; // aff store should assign this
   
      
   msg.aff.otdf_type = otdf_type;
   msg.aff.aff_store_control = msg.aff.NEW;
   
   //map<string, double >::iterator obj_it = instance_in->params_map_.find("x");
   msg.aff.origin_xyz[0] =instance_in->params_map_.find("x")->second;
   msg.aff.origin_xyz[1] =instance_in->params_map_.find("y")->second;
   msg.aff.origin_xyz[2] =instance_in->params_map_.find("z")->second;
   msg.aff.origin_rpy[0] =instance_in->params_map_.find("roll")->second;
   msg.aff.origin_rpy[1] =instance_in->params_map_.find("pitch")->second;
   msg.aff.origin_rpy[2] =instance_in->params_map_.find("yaw")->second;
   
   double bounding_xyz[]={0,0,0};
   double bounding_rpy[]={0,0,0};
   double bounding_lwh[]={0,0,0};

   if(otdf_type == "car"){
      //TODO centralize these settings.  This is duplicated in segmentation code
      msg.aff.modelfile = "car.pcd";
      bounding_xyz[2] = 1.0;   // center of bounding box is 1m above car origin
      bounding_lwh[0] = 3.0;
      bounding_lwh[1] = 1.7;
      bounding_lwh[2] = 2.2;
   } else commonShapeBoundingBox(otdf_type, instance_in, bounding_xyz, bounding_rpy, bounding_lwh);

   msg.aff.bounding_xyz[0] = bounding_xyz[0]; msg.aff.bounding_xyz[1] = bounding_xyz[1];msg.aff.bounding_xyz[2] = bounding_xyz[2];
   msg.aff.bounding_rpy[0] = bounding_rpy[0]; msg.aff.bounding_rpy[1] = bounding_rpy[1];msg.aff.bounding_rpy[2] = bounding_rpy[2];
   msg.aff.bounding_lwh[0] = bounding_lwh[0]; msg.aff.bounding_lwh[1] = bounding_lwh[1];msg.aff.bounding_lwh[2] = bounding_lwh[2];

  
   typedef std::map<std::string, double > params_mapType;
   for( params_mapType::const_iterator it = instance_in->params_map_.begin(); it!=instance_in->params_map_.end(); it++)
   {
      // don't copy xyz ypr
      if(isRedundantParam(it->first)) continue;
      // copy all other params
      msg.aff.param_names.push_back(it->first);
      msg.aff.params.push_back(it->second);
   }
   msg.aff.nparams =  msg.aff.param_names.size();

  int cnt=0;
   typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
    for (joints_mapType::iterator it = instance_in->joints_.begin();it != instance_in->joints_.end(); it++)
    {   
      if(it->second->type!=(int) otdf::Joint::FIXED) {

          double pos, vel;
          instance_in->getJointState(it->first,pos,vel);
          cnt++;
          msg.aff.state_names.push_back(it->first);
          msg.aff.states.push_back(pos);
      }
     }
   msg.aff.nstates =  cnt;
   msg.npoints = 0;
   msg.ntriangles = 0;
   cout <<"publish_otdf_instance_to_affstore: creating a new instance of template :" << msg.aff.otdf_type << endl;
    
   _lcm->publish(channel, &msg);
  } 
 //-------------------------------------------------------------------------------------------   


  
