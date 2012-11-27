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
  
  /**Subscribes to Robot URDF Model and to EST_ROBOT_STATE.*/
  RobotStateListener::RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer):
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
    _urdf_subscription = lcm->subscribe("ROBOT_MODEL", 
				       &fk::RobotStateListener::handleRobotUrdfMsg,
				       this);    
    _urdf_subscription_on = true;
    //Subscribes to MEAS_JOINT_ANGLES 
    //lcm->subscribe("MEAS_JOINT_ANGLES", &fk::RobotStateListener::handleJointAnglesMsg, this); //&this ?
    lcm->subscribe("EST_ROBOT_STATE", &fk::RobotStateListener::handleRobotStateMsg, this); //&this ?


  }
  


  RobotStateListener::~RobotStateListener() {}


  //=============message callbacks

  //void RobotStateListener::handleJointAnglesMsg(const lcm::ReceiveBuffer* rbuf,
//						 const string& chan, 
//						 const drc::joint_angles_t* msg)
void RobotStateListener::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_state_t* msg)						 
  {

   //int64_t tic = bot_timestamp_now();
    if (!_urdf_parsed)
      {
	//cout << "\n handleRobotStateMsg: Waiting for urdf to be parsed" << endl;
	return;
      }
    if(_urdf_subscription_on)
     {
       cout << "\n handleRobotStateMsg: unsubscribing from _urdf_subscription" << endl;
       _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
	 _urdf_subscription_on =  false; 	
    }
  
    //clear stored data
    _link_names.clear();
    _link_tfs.clear();
    _link_shapes.clear();    

    // call a routine that calculates the transforms the joint_state_t* msg.
    map<string, double> jointpos_in;
    for (uint i=0; i< (uint) msg->num_joints; i++) //cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(msg->joint_name[i], msg->joint_position[i]));  
    
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
   
    typedef map<string, shared_ptr<urdf::Link> > links_mapType;
    for( links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
      {  		
        
	if(it->second->visual)
	  {
	 
	    urdf::Pose visual_origin = it->second->visual->origin;
	     
	    KDL::Frame T_parentjoint_visual, T_body_parentjoint, T_body_visual, T_world_body, T_world_visual;

	    T_world_body.p[0]= msg->origin_position.translation.x;
	    T_world_body.p[1]= msg->origin_position.translation.y;
	    T_world_body.p[2]= msg->origin_position.translation.z;		    
	    T_world_body.M =  KDL::Rotation::Quaternion(msg->origin_position.rotation.x, msg->origin_position.rotation.y, msg->origin_position.rotation.z, msg->origin_position.rotation.w);
	    
	    map<string, drc::transform_t>::const_iterator transform_it;
	    transform_it=cartpos_out.find(it->first);
        
           // usually find fails if base_link has a visual element.
	   // Kdl based FK ignores the root link which must be set to body pose
	   // manually.
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
	      
	       
	    
	    //state.tf.translation = transform_it->second.translation;
	    //state.tf.rotation = transform_it->second.rotation;
	    
	    //cout << "\nlink_name : " << it->first << endl; 
	    //cout << "timestamp  : " << msg->utime << endl;    
	    shared_ptr<urdf::Geometry> geom =  it->second->visual->geometry;

	    //---store
	    _link_names.push_back(it->first);
	    _link_shapes.push_back(geom);
	    _link_tfs.push_back(state);
	    //---

	    //cout << "translation  : " << endl;
	    //cout << "\t .x  : " << state.tf.translation.x << endl;
	    //cout << "\t .y  : " << state.tf.translation.y << endl;
	    //cout << "\t .z  : " << state.tf.translation.z << endl;
	    //cout << "quaternion" << endl;
	    //cout << "\t .x  : " << state.tf.rotation.x << endl;
	    //cout << "\t .y  : " << state.tf.rotation.y << endl;
	    //cout << "\t .z  : " << state.tf.rotation.z << endl;
	    //cout << "\t .w  : " << state.tf.rotation.w << endl;   
	    //cout << "\n"<< endl;
	    
	    }//if(transform_it!=cartpos_out.end())
	    else // root link has visual element (but is not part of fk output)
	    {
	   
	   
		urdf::Pose visual_origin = it->second->visual->origin;
		T_body_visual.p[0]=visual_origin.position.x;
		T_body_visual.p[1]=visual_origin.position.y;
		T_body_visual.p[2]=visual_origin.position.z;
		T_body_visual.M =  KDL::Rotation::Quaternion(visual_origin.rotation.x, visual_origin.rotation.y, visual_origin.rotation.z, visual_origin.rotation.w);

		   
		T_world_visual = T_world_body*T_body_visual;


		   
		shared_ptr<urdf::Geometry> geom =  it->second->visual->geometry;
		drc::link_transform_t state;

		state.link_name = it->first;
	
		// For Body Frame Viewing
		//state.tf.translation.x = T_body_visual.p[0];
		//state.tf.translation.y = T_body_visual.p[1];
		//state.tf.translation.z = T_body_visual.p[2];
		//T_body_visual.M.GetQuaternion(state.tf.rotation.x,state.tf.rotation.y,state.tf.rotation.z,state.tf.rotation.w);
	
		state.tf.translation.x = T_world_visual.p[0];
		state.tf.translation.y = T_world_visual.p[1];
		state.tf.translation.z = T_world_visual.p[2];
		T_world_visual.M.GetQuaternion(state.tf.rotation.x,state.tf.rotation.y,state.tf.rotation.z,state.tf.rotation.w);

		//---store
		_link_names.push_back(it->first);
		_link_shapes.push_back(geom);
		_link_tfs.push_back(state);

	    }

	  }//if(it->second->visual)
   
      }//end for

    bot_viewer_request_redraw(_viewer);
 //int64_t toc = bot_timestamp_now();
 //cout << bot_timestamp_useconds(toc-tic) << endl;
  } // end handleMessage

  
  void RobotStateListener::handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const string& channel, 
					       const  drc::robot_urdf_t* msg) 
  {

    if(_urdf_parsed ==false) 
    {
     cout<< "\nurdf handler @ RobotStateListener" << endl;
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
   
   // Must also add the pose of the camera link "wide_stereo_link".
   
     
   //robot_model.getLinks(robot->links_); 
   _links_map =  robot_model.links_;

   // loop through all the links
    typedef map<string, shared_ptr<urdf::Link> > links_mapType;
    for(links_mapType::const_iterator it =  _links_map.begin(); it!= _links_map.end(); it++)
     { 
	if(it->second->visual){
		cout << it->first << endl;
	  int type = it->second->visual->geometry->type;

	    enum {SPHERE, BOX, CYLINDER, MESH}; 
	   
	   if  (type == MESH){
	      shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(it->second->visual->geometry));
	      
	      // READ AND STORE DISPLAY LISTS IN MEMORY ONCE
    		string file_path = evalMeshFilePath(mesh->filename);
		cout <<"MESH:" << file_path << endl;
		BotWavefrontModel *wavefront_model;
		wavefront_model = bot_wavefront_model_create(file_path.c_str());

		GLuint dl = glGenLists (1);
		glNewList (dl, GL_COMPILE);
		//glEnable(GL_LIGHTING);
		bot_wavefront_model_gl_draw(wavefront_model);
		//glDisable(GL_LIGHTING);
		glEndList ();
		double minv[3];
		double maxv[3];
		bot_wavefront_model_get_extrema(wavefront_model, minv, maxv);
    MeshExtrema ext_struct;
		ext_struct.span_x = maxv[0] - minv[0];
		ext_struct.span_y = maxv[1] - minv[1];
		ext_struct.span_z = maxv[2] - minv[2];
		ext_struct.offset_x = (maxv[0] + minv[0])/2.0;
		ext_struct.offset_y = (maxv[1] + minv[1])/2.0;
		ext_struct.offset_z = (maxv[2] + minv[2])/2.0;
		bot_wavefront_model_destroy(wavefront_model);
		_mesh_map.insert(make_pair(it->first, dl)); 
    _mesh_extrema_map.insert(make_pair(it->first, ext_struct)); 
	    }
	}
     }
   //---------parse the tree and stop listening for urdf messages

   // Parse KDL tree
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(_urdf_xml_string,tree))
      {
	cerr << "ERROR: Failed to extract kdl tree from xml robot description" << endl; 
	return;
      }

    //unsubscribe from urdf messages
    //_lcm->unsubscribe(_urdf_subscription);  // crashes viewer if there are other urdf subscriptions in other renderers.
    
    //
    _fksolver = shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));
    //remember that we've parsed the urdf already
    _urdf_parsed = true;

    cout<< "Number of Joints: " << _joint_names_.size() <<endl;
    
    }//  if(_urdf_parsed ==false)   
  } 

  
  //================printouts
  void RobotStateListener::getState(vector<shared_ptr<urdf::Geometry> > & shapes,vector<drc::link_transform_t> & transforms, std::vector<std::string> & names)
    {
      names = _link_names;
      shapes = _link_shapes;
      transforms = _link_tfs;
      
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

  void RobotStateListener::printTransforms(const vector<shared_ptr<urdf::Geometry> > &_link_shapes,
					   const vector<drc::link_transform_t> &_link_tfs)
  {
    for (uint i = 0; i < _link_tfs.size(); i++)
      {
	drc::link_transform_t next = _link_tfs[i];
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

std::string RobotStateListener::evalMeshFilePath(std::string file_path_expression) {
    std::string result = "";
    std::string package_path = std::string(getModelsPath()) + "/mit_gazebo_models/"; // getModelsPath gives /drc/software/build/models/
    std::string token_str1 ("package://");
    std::string token_str2 (".");
    size_t found1, found2;
    std::string  file_path= "";
    found1=file_path_expression.find(token_str1);
    if (found1!=std::string::npos) // if string contains package:// as a substring 
    {  
	found2=file_path_expression.find(token_str2);
        file_path = package_path + file_path_expression.substr(found1+token_str1.size(),found2-found1-token_str1.size())+".obj"; 
        //file_path = package_path + file_path_expression.substr(found1+token_str1.size(),found2-found1-token_str1.size())+"_chull.obj"; 
    }
    else
    	file_path = file_path_expression;


    return file_path;
}


std::string RobotStateListener::exec(std::string cmd) {
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
    	if(fgets(buffer, 128, pipe) != NULL)
    		result += buffer;
    }
    pclose(pipe);
    return result;
}

std::string RobotStateListener::evalROSMeshFilePath(std::string file_path_expression) {
    std::string result = "";
    std::string token_str1 ("package://");
    std::string token_str2 ("/");
    size_t found1, found2;
    std::string  file_path= "";
    found1=file_path_expression.find(token_str1);
    if (found1!=std::string::npos) // if string contains package:// as a substring 
    {
     	found2 = file_path_expression.find(token_str2,found1+token_str1.size()+1);
     	std::string package_name = file_path_expression.substr(found1+token_str1.size(),found2-found1-token_str1.size());

     	std::string cmd = "rospack find " + package_name;  
     	std::string  package_path = exec(cmd);

     	file_path = package_path.substr(0,package_path.size()-1) + 	    file_path_expression.substr(found2); 	
    }
    else
    	file_path = file_path_expression;

    return file_path;
}


} //namespace fk


