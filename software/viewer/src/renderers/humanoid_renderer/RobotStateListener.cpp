// file: test_visual_elements_extractor.cpp
// This file links to treefksolverposfull_recursive.cpp thats provides a routine to solve forward kinematics 
// upon receipt of a joint_angles_t message.
// for the whole kinematic tree. KDL lib functions only provide an interface to query global position between
// a specified root and a tip segment.


#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "RobotStateListener.hpp"

using namespace std;
using namespace boost;

namespace fk 
{
  //==================constructor / destructor
  
  /**Subscribes to Robot URDF Model and to MEAS_JOINT_ANGLES.*/
  RobotStateListener::RobotStateListener(lcm::LCM &lcm):
    _urdf_parsed(false),
    _lcm(lcm)
  {
    //lcm ok?
    if(!lcm.good())
      {
	cerr << "\nLCM Not Good: Joint Angles Handler" << endl;
	return;
      }
    
    // Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
    _urdf_subscription = lcm.subscribe("ROBOT_MODEL", 
				       &fk::RobotStateListener::handleRobotUrdfMsg,
				       this);    
    //Subscribes to MEAS_JOINT_ANGLES 
    lcm.subscribe("MEAS_JOINT_ANGLES", &fk::RobotStateListener::handleJointAnglesMsg, this); //&this ?
  }
  


  RobotStateListener::~RobotStateListener() {}


  //=============message callbacks

  void RobotStateListener::handleJointAnglesMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::joint_angles_t* msg)
  {
    if (!_urdf_parsed)
      {
	cout << "\n handleJointAnglesMsg: Waiting for urdf to be parsed" << endl;
	return;
      }

    // call a routine that calculates the transforms the joint_state_t* msg.
    map<string, double> jointpos_in;
    for (uint i=0; i< msg->num_joints; i++)
      jointpos_in.insert(make_pair(msg->joint_name[i], msg->angular_position[i]));  
    
    map<string, drc::transform_t > cartpos_out;
    
		  // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree=true; // determines the absolute transforms with respect to robot origin. 
                            //Otherwise returns relative transforms between joints. 
    
    kinematics_status = _fksolver->JntToCart(jointpos_in,cartpos_out,flatten_tree);
    
    if(kinematics_status>=0){
      // cout << "Success!" <<endl;
    }else{
       cerr << "Error: could not calculate forward kinematics!" <<endl;
       return;
   }
    
    // PRINTS THE VISUAL PROPERTIES OF ALL LINKS THAT HAVE A VISUAL ELEMENT DEFINED IN THE URDF FILE 
    map<string, drc::transform_t>::const_iterator transform_it;
    typedef map<string, shared_ptr<urdf::Link> > links_mapType;
    for( links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
      {  		

	if(it->second->visual)
	  {
	    
	    urdf::Pose visual_origin = it->second->visual->origin;
	    
	    KDL::Frame T_visualorigin_parentjoint, T_parentjoint_bodyorigin, T_visualorigin_bodyorigin;
	    
	    T_visualorigin_parentjoint.p[0]=visual_origin.position.x;
	    T_visualorigin_parentjoint.p[1]=visual_origin.position.y;
	    T_visualorigin_parentjoint.p[2]=visual_origin.position.z;
	    T_visualorigin_parentjoint.M.Quaternion(visual_origin.rotation.x,visual_origin.rotation.y,visual_origin.rotation.z,visual_origin.rotation.w);
	    
	    transform_it=cartpos_out.find(it->first);
	    
	    T_parentjoint_bodyorigin.p[0]= transform_it->second.translation.x;
	    T_parentjoint_bodyorigin.p[1]= transform_it->second.translation.y;
	    T_parentjoint_bodyorigin.p[2]= transform_it->second.translation.z;			
	    T_parentjoint_bodyorigin.M.Quaternion(transform_it->second.rotation.x,transform_it->second.rotation.y,transform_it->second.rotation.z,transform_it->second.rotation.w);
	    
	    T_visualorigin_bodyorigin = T_visualorigin_parentjoint*T_parentjoint_bodyorigin; 
	    
	    drc::link_transform_t state;	    
	    
	    state.link_name = transform_it->first;
	    
	    state.tf.translation.x = T_visualorigin_bodyorigin.p[0];
	    state.tf.translation.y = T_visualorigin_bodyorigin.p[1];
	    state.tf.translation.z = T_visualorigin_bodyorigin.p[2];
	    T_visualorigin_bodyorigin.M.GetQuaternion(state.tf.rotation.x,state.tf.rotation.y,state.tf.rotation.z,state.tf.rotation.w);
	    
	    //state.tf.translation = transform_it->second.translation;
	    //state.tf.rotation = transform_it->second.rotation;
	    
	    cout << "link_name : " << it->first <<"\n"<< endl; 
	    cout << "timestamp  : " << msg->timestamp << endl;    
	    shared_ptr<urdf::Geometry> geom =  it->second->visual->geometry;
	    int type = it->second->visual->geometry->type ;
	    enum {SPHERE, BOX, CYLINDER, MESH}; 
	    
	    if (type == SPHERE){
	      cout << "SPHERE"<< endl;
	      shared_ptr<urdf::Sphere> sphere(shared_dynamic_cast<urdf::Sphere>(it->second->visual->geometry));	
	      cout << "radius : "<<  sphere->radius << endl;
	      // drawSphere(radius, it->second -> visual ->origin);
	    }
	    else if  (type == BOX){
	      shared_ptr<urdf::Box> box(shared_dynamic_cast<urdf::Box>(it->second->visual->geometry));
	      cout << "BOX"<< endl;
	      cout << "dim.x : "<<  box->dim.x << endl;
	      cout << "dim.y : "<<  box->dim.y << endl;
	      cout << "dim.z : "<<  box->dim.z << endl;
	      // drawBox(dim, it->second -> visual->origin);
	    }
	    else if  (type == CYLINDER){
	      shared_ptr<urdf::Cylinder> cyl(shared_dynamic_cast<urdf::Cylinder>(it->second->visual->geometry));
	      cout << "CYLINDER"<< endl;
	      cout << "radius : "<<  cyl->radius << endl;
	      cout << "length : "<<  cyl->length << endl;
	      // drawBox(radius,length, it->second -> visual->origin);
	    }
	    else if  (type == MESH){
	      cout << "MESH"<< endl;
	      //shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(it->second->visual->geometry));
	      //renderMesh(mesh->filename)
	    }
	    else {
	      cout << "UNKNOWN"<< endl;
	    }
	    
	    cout << "translation  : " << endl;
	    cout << "\t .x  : " << state.tf.translation.x << endl;
	    cout << "\t .y  : " << state.tf.translation.y << endl;
	    cout << "\t .z  : " << state.tf.translation.z << endl;
	    cout << "quaternion" << endl;
	    cout << "\t .x  : " << state.tf.rotation.x << endl;
	    cout << "\t .y  : " << state.tf.rotation.y << endl;
	    cout << "\t .z  : " << state.tf.rotation.z << endl;
	    cout << "\t .w  : " << state.tf.rotation.w << endl;   
	    cout << "\n"<< endl;
	    
	  }//if(it->second->visual)
      }//end for
  } // end handleMessage

  
  void RobotStateListener::handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::robot_urdf_t* msg) 
  {
    // Received robot urdf string. Store it internally and get all available joints.
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

    //unsubscribe from urdf messages
    _lcm.unsubscribe(_urdf_subscription); 
    
    //
    _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
    //remember that we've parsed the urdf already
    _urdf_parsed = true;

    cout<< "Number of Joints: " << _joint_names_.size() <<endl;
} 


} //namespace fk


