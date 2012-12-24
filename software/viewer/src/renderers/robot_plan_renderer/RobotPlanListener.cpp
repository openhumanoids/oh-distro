// file: test_visual_elements_extractor.cpp
// This file links to treefksolverposfull_recursive.cpp thats provides a routine to solve forward kinematics 
// upon receipt of a joint_angles_t message.
// for the whole kinematic tree. KDL lib functions only provide an interface to query global position between
// a specified root and a tip segment.


#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "RobotPlanListener.hpp"

using namespace std;
using namespace boost;

namespace fk 
{
  //==================constructor / destructor
  
  /**Subscribes to Robot URDF Model and to EST_ROBOT_STATE.*/
  RobotPlanListener::RobotPlanListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer):
    _urdf_parsed(false),
    _lcm(lcm),
    _viewer(viewer)

  {
    //lcm ok?
    if(!lcm->good())
      {
	cerr << "\nLCM Not Good: Robot State Handler" << endl;
	return;
      }

    // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
    _urdf_subscription = _lcm->subscribe("ROBOT_MODEL", 
				       &fk::RobotPlanListener::handleRobotUrdfMsg,
				       this);  
    _urdf_subscription_on =  true;
    lcm->subscribe("CANDIDATE_ROBOT_PLAN", &fk::RobotPlanListener::handleRobotPlanMsg, this); //&this ?
    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
  }
  


  RobotPlanListener::~RobotPlanListener() {}


  //=============message callbacks

void RobotPlanListener::handleRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_plan_t* msg)						 
  {
    if (!_urdf_parsed)
      {
	cout << "\n handleRobotPlanMsg: Waiting for urdf to be parsed" << endl;
	return;
      }
   if(_urdf_subscription_on)
     {
       cout << "\n handleRobotPlanMsg: unsubscribing from _urdf_subscription" << endl;
       _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
	 _urdf_subscription_on =  false; 	
    }
   
   
    //clear stored data
    _link_tfs.clear();
    _link_shapes.clear();    
    
  for (uint i = 0; i <(uint)msg->num_states; i++)
	{
    drc::robot_state_t state_msg  = msg->plan[i];
    

    // call a routine that calculates the transforms the robot_state_t* msg.
    map<string, double> jointpos_in;
    for (uint i=0; i< (uint) state_msg.num_joints; i++) { //cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(state_msg.joint_name[i], state_msg.joint_position[i]));
    }
   
    map<string, drc::transform_t > cartpos_out;
    
		  // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. 
                            //Otherwise returns relative transforms between joints. 
    
    kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
    if(kinematics_status<0){
      cerr << "Error: could not calculate forward kinematics!" <<endl;
      return;
    }

    // PRINTS THE VISUAL PROPERTIES OF ALL LINKS THAT HAVE A VISUAL ELEMENT DEFINED IN THE URDF FILE 
   
    typedef map<string, shared_ptr<urdf::Link> > links_mapType;
    for( links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
      {  		

	      if(it->second->visual)
	        {
	         
	          urdf::Pose visual_origin = it->second->visual->origin;
	           
	          KDL::Frame T_parentjoint_visual, T_body_parentjoint, T_body_visual, T_world_body, T_world_visual;
	          
	          
            double norm_quarternion = sqrt(pow(state_msg.origin_position.rotation.x,2)+pow(state_msg.origin_position.rotation.y,2)+pow(state_msg.origin_position.rotation.z,2)+pow(state_msg.origin_position.rotation.w,2));
	          	          
	          if (norm_quarternion==1){

	          T_world_body.p[0]= state_msg.origin_position.translation.x;
	          T_world_body.p[1]= state_msg.origin_position.translation.y;
	          T_world_body.p[2]= state_msg.origin_position.translation.z;		    
	          T_world_body.M =  KDL::Rotation::Quaternion(state_msg.origin_position.rotation.x, state_msg.origin_position.rotation.y, state_msg.origin_position.rotation.z, state_msg.origin_position.rotation.w);
	          }
	          else{
	           std::cout <<"ERROR: Improper body origin quaternion, norm is not 1. Using identity instead." <<std::endl;
	          T_world_body.p[0]= 0;
	          T_world_body.p[1]= 0;
	          T_world_body.p[2]= 0;		    
	          T_world_body.M =  KDL::Rotation::Quaternion(0,0,0,1);
	          }
	          
	          map<string, drc::transform_t>::const_iterator transform_it;
	          transform_it=cartpos_out.find(it->first);  
              
           // usually find fails if base_link has a visual element.
	         // Kdl based FK ignores the root link which must be set to body pose
	         // manually (TODO).
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

	            // For Body Frame Viewing
	            //state.tf.translation.x = T_body_visual.p[0];
	            //state.tf.translation.y = T_body_visual.p[1];
	            //state.tf.translation.z = T_body_visual.p[2];
	            //T_body_visual.M.GetQuaternion(state.tf.rotation.x,state.tf.rotation.y,state.tf.rotation.z,state.tf.rotation.w);
	
	            state.tf.translation.x = T_world_visual.p[0];
	            state.tf.translation.y = T_world_visual.p[1];
	            state.tf.translation.z = T_world_visual.p[2];
	            T_world_visual.M.GetQuaternion(state.tf.rotation.x,state.tf.rotation.y,state.tf.rotation.z,state.tf.rotation.w);
	            
	            shared_ptr<urdf::Geometry> geom =  it->second->visual->geometry;

	            //---store
	            _link_shapes.push_back(geom);
	            _link_tfs.push_back(state);	    
	            
	          }//if(transform_it!=cartpos_out.end())
	        }//if(it->second->visual)
      }//end for all links in _links_map (created from urdf) 
      
    }//end for num of states in robot_plan msg;
    _last_plan_msg_timestamp = bot_timestamp_now(); //initialize
    bot_viewer_request_redraw(_viewer);
  } // end handleMessage

/*  void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::robot_urdf_t* msg, RobotPlanListener* listener) 
 */
  
 void RobotPlanListener::handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::robot_urdf_t* msg) 
  
{

    if(_urdf_parsed ==false) 
    {
     // Received robot urdf string. Store it internally and get all available joints.
      cout<< "\nurdf handler @ RobotPlanListener" << endl;
    _robot_name      = msg->robot_name;
    _urdf_xml_string = msg->urdf_xml_string;
    cout<< "\nReceived urdf_xml_string of robot [" 
	<< msg->robot_name << "], storing it internally as a param" << endl;
  
	
    // Get a urdf Model from the xml string and get all the joint names.
    urdf::Model robot_model; 
    if (!robot_model.initString( msg->urdf_xml_string))
      {
       cerr << "ERROR: Could not generate robot model" << endl;
     }

   typedef map<string, shared_ptr<urdf::Joint> > joints_mapType;
   for( joints_mapType::const_iterator it = robot_model.joints_.begin(); it!=robot_model.joints_.end(); it++)
     { 
       if(it->second->type!=6) // All joints that not of the type FIXED.
	 _joint_names_.push_back(it->first);
     }
   
   // Must also add the pose of the camera link "wide_stereo_link".
   
     
   //robot_model.getLinks(robot->links_); 
   _links_map =  robot_model.links_;
   
   //---------parse the tree and stop listening for urdf messages

   // Parse KDL tree
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
      {
	cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl; 
	return;
      }
 
    //unsubscribe from urdf messages (Viewer crashes)
    //_lcm->unsubscribe(_urdf_subscription); 
    
    //
    _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
    //remember that we've parsed the urdf already
    _urdf_parsed = true;

    cout<< "Number of Joints: " << _joint_names_.size() <<endl;
    
    }//  if(_urdf_parsed ==false) 

  } 

  
  //================printouts
  void RobotPlanListener::getState(vector<shared_ptr<urdf::Geometry> > & shapes,
				    vector<drc::link_transform_t> & transforms)
    {
      shapes = _link_shapes;
      transforms = _link_tfs;
      
      // for debugging
      //printTransforms(_link_shapes,_link_tfs);
      
      //todo: why does the code below seg fault?
      /*
      shapes.clear();
      transforms.clear();

      if (_link_shapes.size() == 0 || _link_tfs.size() == 0)
	{
	  cout << "\n waiting for link/shapes to have non-zero size" << endl;
	  return;
	}
      
      
      if (_link_shapes.size() != _link_tfs.size())
	{
	  cerr << "\n\nError: why aren't _link_shapes and _link_tfs of the same size" <<endl;	
	  cout << "\n\n\nError: why aren't _link_shapes and _link_tfs of the same size\n\n\n" <<endl;	
	  return;
	}

      for (uint i = 0; i < _link_shapes.size(); i++)
	{
	  shapes[i] = _link_shapes[i];
	  transforms[i] = _link_tfs[i];
	}
      */
    }

  void RobotPlanListener::printTransforms(const vector<shared_ptr<urdf::Geometry> > &_link_shapes,
					   const vector<drc::link_transform_t> &_link_tfs)
  {
    for (uint i = 0; i < _link_tfs.size(); i++)
      {
	drc::link_transform_t next = _link_tfs[i];
	cout << "geom type (SPHERE-0, BOX-1, CYLINDER-2, MESH-3}) : \n" <<_link_shapes[i]->type<< endl;
	cout << "translation  : " << endl;
	cout << "\t .x  : " << next.tf.translation.x << endl;
	cout << "\t .y  : " << next.tf.translation.y << endl;
	cout << "\t .z  : " << next.tf.translation.z << endl;
	cout << "quaternion" << endl;
	cout << "\t .x  : " << next.tf.rotation.x << endl;
	cout << "\t .y  : " << next.tf.rotation.y << endl;
	cout << "\t .z  : " << next.tf.rotation.z << endl;
	cout << "\t .w  : " << next.tf.rotation.w << endl;   
	cout << "\n"<< endl;
      }

  }

} //namespace fk


