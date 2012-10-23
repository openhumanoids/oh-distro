
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "AffordanceCollectionListener.hpp"

using namespace std;
using namespace boost;

namespace otdf_renderer
{
  //==================constructor / destructor

//   AffordanceCollectionListener::AffordanceCollectionListener(boost::shared_ptr<lcm::LCM> &lcm, RendererOtdf* otdf_renderer):
//     _lcm(lcm),
//     _parent_otdf_renderer(otdf_renderer)
  AffordanceCollectionListener::AffordanceCollectionListener( RendererOtdf* otdf_renderer):
    _parent_otdf_renderer(otdf_renderer)
  {
    _lcm = otdf_renderer->lcm;
    
 
    //lcm ok?
    if(!_lcm->good())
      {
	cerr << "\nLCM Not Good: Robot State Handler" << endl;
	return;
      }


    _lcm->subscribe("AFFORDANCE_COLLECTION", &AffordanceCollectionListener::handleAffordanceCollectionMsg, this); 
    _lcm->subscribe("AFFORDANCE", &AffordanceCollectionListener::handleAffordanceMsg, this); 

  }
  


  AffordanceCollectionListener::~AffordanceCollectionListener() {}

 // =======================================================================  
 //  message callbacks
 // =======================================================================   

// checks to see the affordances are in the parent renderer object list. If it is pre-existing 
// it is updated and if its new a new instance is created.
void AffordanceCollectionListener::handleAffordanceCollectionMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::affordance_collection_t* msg)						 
  {
    
    //cout << "Ok!: " << msg->name << endl;
  
    for (size_t i=0; i< (size_t)msg->naffs; i++)
    {
      const drc::affordance_t aff = msg->affs[i];

        
      typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
      object_instance_map_type_::iterator it = _parent_otdf_renderer->instantiated_objects.find(aff.name);
       
       if (it!=_parent_otdf_renderer->instantiated_objects.end()) {
          //exists so update
          cout <<"updated_otdf_object_instance: "<< aff.name << endl;
          update_object_instance(aff);
       }      
       else{
          // add new otdf instance
          std::string filename = get_filename(aff.otdf_id);
          cout <<"add new otdf instance: "<< aff.name << ", of template :" << filename << endl;
          //std::transform(filename.begin(), filename.end(), filename.begin(), ::tolower);
          add_new_otdf_object_instance(filename, aff);
       }
        
    }

  } // end handleMessage

   // =======================================================================  
   
  void AffordanceCollectionListener::handleAffordanceMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::affordance_t* msg) 
  {
       cout << "Received affordance: " << msg->name << endl;
       
       typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
       object_instance_map_type_::iterator it = _parent_otdf_renderer->instantiated_objects.find(msg->name);
       if (it!=_parent_otdf_renderer->instantiated_objects.end()) {
          //exists so update
         cout <<"update_otdf_object_instance" << endl;
         update_object_instance((*msg));
       }      
       else{
          // add new otdf instance
          cout <<"add new otdf instance" << endl;
          std::string filename = get_filename(msg->otdf_id);
          add_new_otdf_object_instance(filename, (*msg));
       }
       
  } 

 // =======================================================================  
 //  Utils
 // =======================================================================   

 std::string AffordanceCollectionListener::get_filename(int32_t otdf_id)
 {
   enum{CYLINDER, LEVER};
       
  std::string filename;
          
  //NOTE: Have to manually list all the otdf templates, not ideal. 
  //Much better if otdf_id is a string instead of a enumerated type.
  if(otdf_id==CYLINDER){
   // check if cylinder exists in _parent_otdf_renderer->otdf_filenames
   filename = "cylinder";
  }
  else if(otdf_id==LEVER)
  {
   filename = "lever";
  }
    
  return filename;
 }
 // =======================================================================
void AffordanceCollectionListener::add_new_otdf_object_instance (std::string &filename, const drc::affordance_t &aff)
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
  // Parse KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
  {
    std::cerr << "ERROR: Failed to extract kdl tree from "  << filename << " xml object description "<< std::endl; 
  }
  
  instance_struc._fksolver = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

       
   //set All Params
   for (size_t i=0; i < (size_t)aff.nparams; i++)
   {   
       instance_struc._otdf_instance->setParam(aff.param_names[i],aff.params[i]);   
   }
   instance_struc._otdf_instance->update();
  
  
  run_fk_and_gen_link_shapes_and_tfs(instance_struc);

  
  std::map<std::string, int >::iterator it;
  it= _parent_otdf_renderer->instance_cnt.find(filename);
  it->second = it->second + 1;
  std::stringstream oss;
  oss << filename << "_"<< it->second; //TODO: eventually use object_id? 
  //_parent_otdf_renderer->instantiated_objects.insert(std::make_pair(oss.str(), instance_struc));
  _parent_otdf_renderer->instantiated_objects.insert(std::make_pair(aff.name, instance_struc));
  // Update params 
  
  //Request redraw
  bot_viewer_request_redraw(_parent_otdf_renderer->viewer);
} 


// =======================================================================  
void AffordanceCollectionListener::update_object_instance (const drc::affordance_t &aff)
{
  typedef std::map<std::string, OtdfInstanceStruc > object_instance_map_type_;
       object_instance_map_type_::iterator it = _parent_otdf_renderer->instantiated_objects.find(aff.name);
       
   //set All Params
   for (size_t i=0; i < (size_t)aff.nparams; i++)
   {   
       it->second._otdf_instance->setParam(aff.param_names[i],aff.params[i]);   
   }

    it->second._otdf_instance->update();
      std::string _urdf_xml_string = otdf::convertObjectInstanceToCompliantURDFstring(it->second._otdf_instance);
      // Parse KDL tree
      // TODO: create a KDL tree parser from OTDF instance, without having to convert to urdf.
      // otdf can contain some elements that are not part of urdf. e.g. TORUS
      KDL::Tree tree;
      if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
      {
        std::cerr << "ERROR: Failed to extract kdl tree from "  << it->second._otdf_instance->getName() << " xml object description "<< std::endl; 
      }
      
      it->second._fksolver = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
      run_fk_and_gen_link_shapes_and_tfs(it->second);
      
      //Request redraw
    bot_viewer_request_redraw(_parent_otdf_renderer->viewer);
 }
 
 // =======================================================================
  void AffordanceCollectionListener::run_fk_and_gen_link_shapes_and_tfs (OtdfInstanceStruc &instance_struc)
  {

      //clear stored data
      instance_struc._link_tfs.clear();
      instance_struc._link_shapes.clear();
      
      std::map<std::string, double> jointpos_in;
      enum  {UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED};
   
    
     typedef std::map<std::string,boost::shared_ptr<otdf::Joint> > joints_mapType;
     for (joints_mapType::iterator joint = instance_struc._otdf_instance->joints_.begin();joint != instance_struc._otdf_instance->joints_.end(); joint++)
     {

         if(joint->second->type!=(int) FIXED) { // All joints that not of the type FIXED.
            double dof_current_pos = 0; // TODO: need object's initial dof state 
            jointpos_in.insert(make_pair(joint->first, dof_current_pos)); 
         }
     }
     
    //Have to handle joint_patterns separately   
    // DoF of all joints in joint patterns.
    typedef std::map<std::string,boost::shared_ptr<otdf::Joint_pattern> > jp_mapType;
    for (jp_mapType::iterator jp_it = instance_struc._otdf_instance->joint_patterns_.begin();jp_it != instance_struc._otdf_instance->joint_patterns_.end(); jp_it++)
    {
      // for all joints in joint pattern.
      for (unsigned int i=0; i < jp_it->second->joint_set.size(); i++)
      {

          if(jp_it->second->joint_set[i]->type!=(int) FIXED) { // All joints that not of the type FIXED.
	       double dof_current_pos = 0; //TODO: need object's initial dof state from fitting
              jointpos_in.insert(make_pair(jp_it->second->joint_set[i]->name, dof_current_pos)); 
         } // end if
         
      } // end for all joints in jp
    }// for all joint patterns
      
      std::map<string, drc::transform_t > cartpos_out;
      
		    // Calculate forward position kinematics
      bool kinematics_status;
      bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. 
                              //Otherwise returns relative transforms between joints. 
      
      kinematics_status = instance_struc._fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);

      if(kinematics_status>=0){
        // cout << "Success!" <<endl;
      }
      else{
         std::cerr << "Error: could not calculate forward kinematics!" <<std::endl;
         return;
      }
      
     std::map<std::string, boost::shared_ptr<otdf::Link> > _links_map =  instance_struc._otdf_instance->links_;  // static links
      
    //Have to handle link_patterns separately  
    typedef std::map<std::string,boost::shared_ptr<otdf::Link_pattern> > lp_mapType;
    for (lp_mapType::iterator lp_it = instance_struc._otdf_instance->link_patterns_.begin();lp_it != instance_struc._otdf_instance->link_patterns_.end(); lp_it++)
    {
      // for all joints in joint pattern.
      for (unsigned int i=0; i < lp_it->second->link_set.size(); i++)
      {
        _links_map.insert(std::make_pair(lp_it->second->link_set[i]->name,lp_it->second->link_set[i]));       
      } // end for all links in lp
    }// for all link patterns
    
      typedef std::map<std::string, boost::shared_ptr<otdf::Link> > links_mapType;
      for( links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
      { 
      	if(it->second->visual)
	{
	         
	        otdf::Pose visual_origin = it->second->visual->origin;
          KDL::Frame T_parentjoint_visual, T_body_parentjoint, T_body_visual, T_world_body, T_world_visual;

	        T_world_body.p[0]= instance_struc._otdf_instance->getParam("x");
	        T_world_body.p[1]= instance_struc._otdf_instance->getParam("y");
	        T_world_body.p[2]= instance_struc._otdf_instance->getParam("z");
	        T_world_body.M =  KDL::Rotation::RPY(instance_struc._otdf_instance->getParam("roll"),
	                                             instance_struc._otdf_instance->getParam("pitch"),
	                                             instance_struc._otdf_instance->getParam("yaw"));

	      std::map<std::string, drc::transform_t>::const_iterator transform_it;
	      transform_it=cartpos_out.find(it->first);	  
	         
	         if(transform_it!=cartpos_out.end())// fk cart pos exists
	         {
	            T_body_parentjoint.p[0]= transform_it->second.translation.x;
	            T_body_parentjoint.p[1]= transform_it->second.translation.y;
	            T_body_parentjoint.p[2]= transform_it->second.translation.z;	
                 
	            T_body_parentjoint.M =  KDL::Rotation::Quaternion(transform_it->second.rotation.x, transform_it->second.rotation.y, transform_it->second.rotation.z, transform_it->second.rotation.w);


	            T_parentjoint_visual.p[0]=visual_origin.position.x;
	            T_parentjoint_visual.p[1]=visual_origin.position.y;
	            T_parentjoint_visual.p[2]=visual_origin.position.z;
	            T_parentjoint_visual.M =  KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);

	            T_body_visual  = T_body_parentjoint*T_parentjoint_visual;
	           
	            T_world_visual = T_world_body*T_body_visual;
                     //T_world_visual  = T_world_camera*T_camera_body*T_body_visual;

	            drc::link_transform_t state;	    

	            state.link_name = transform_it->first;

	            state.tf.translation.x = T_world_visual.p[0];
	            state.tf.translation.y = T_world_visual.p[1];
	            state.tf.translation.z = T_world_visual.p[2];
	            T_world_visual.M.GetQuaternion(state.tf.rotation.x,state.tf.rotation.y,state.tf.rotation.z,state.tf.rotation.w);
	              
    
	            boost::shared_ptr<otdf::Geometry> geom =  it->second->visual->geometry;

	            //---store
	            instance_struc._link_shapes.push_back(geom);
	            instance_struc._link_tfs.push_back(state);
	         }  // end if(transform_it!=cartpos_out.end())
	      
	      } // end if(it->second->visual)
         
      } // end for links in  _links_map

  } // end function run_fk_and_gen_link_shapes_and_tfs
  
  
// =======================================================================


} //namespace otdf_renderer


// ========= TYPES ======================================================
//struct affordance_collection_t
//{
//  string name;    // name to display e.g. "kitchen" or "pump room"

//  int64_t map_utime; // utime of the local map we refer to
//  int32_t map_id; // id of the local map - duplication of the above?

//  int32_t naffs;
//  affordance_t affs[naffs];
//}

//struct affordance_t
//{

//  // These are duplication ... remove?
//  int64_t map_utime;
//  int32_t map_id;

//  // which object in the scene is it?
//  int32_t object_id;
//  // which object, see constants below
//  int16_t otdf_id;   
//  // informal label for the affordance:
//  string name;
//   
//  // geomtrical properties, 
//  int32_t nparams;
//  double params[nparams];
//  string param_names[nparams];

//  //   
//  int32_t nstates;
//  double states[nstates];
//  string state_names[nstates];

//  // Point Cloud IDs?

//  const int16_t CYLINDER=0, LEVER=1;
//}
