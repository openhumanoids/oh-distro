#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "RobotStateListener.hpp"

using namespace std;
using namespace boost;

namespace renderer_affordances 
{
  //==================constructor / destructor
  
  /**Subscribes to Robot URDF Model and to EST_ROBOT_STATE.*/
  /*RobotStateListener::RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer):

    _lcm(lcm),
    _viewer(viewer)*/
  RobotStateListener::RobotStateListener(RendererAffordances* parent_renderer):
   _parent_renderer(parent_renderer),_robot_state_received(false),_urdf_parsed(false),object_update_counter(0)
  {

    _lcm = _parent_renderer->lcm; 
    //lcm ok?
    if(!_lcm->good())
    {
      cerr << "\nLCM Not Good: Robot State Handler" << endl;
      return;
    }
    T_body_world = KDL::Frame::Identity();

    _parent_renderer->last_state_msg_timestamp = 0;
     _parent_renderer->robot_name = "atlas";//default
   //(*_parent_renderer->robot_name_ptr) = "atlas"; 

// Subscribe to Robot_Model.  Will unsubscribe once a single message has been received
    _urdf_subscription = _lcm->subscribe("ROBOT_MODEL", 
				       &RobotStateListener::handleRobotUrdfMsg,
				       this);    
    _urdf_subscription_on = true;

    // Subscribe to Robot_state. 
    _lcm->subscribe("EST_ROBOT_STATE", &RobotStateListener::handleRobotStateMsg, this); 
  }
  
  RobotStateListener::~RobotStateListener() {}


  //=============message callbacks

void RobotStateListener::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
						 const string& chan, 
						 const drc::robot_state_t* msg)						 
  { 
	if (!_urdf_parsed)
    {
     //cout << msg->utime << endl;
      //cout << "\n handleRobotStateMsg: Waiting for urdf to be parsed" << endl;
      return;
    }
    if(_urdf_subscription_on)
    {			
      cout << "\n handleRobotStateMsg: unsubscribing from _urdf_subscription" << endl;
      _lcm->unsubscribe(_urdf_subscription);     //unsubscribe from urdf messages
      _urdf_subscription_on =  false; 	
    }

      if(!_robot_state_received)
      _robot_state_received = true;
  
  	  KDL::Frame  T_world_body;
  	    
      T_world_body.p[0]= msg->origin_position.translation.x;
	    T_world_body.p[1]= msg->origin_position.translation.y;
	    T_world_body.p[2]= msg->origin_position.translation.z;		    
	    T_world_body.M =  KDL::Rotation::Quaternion(msg->origin_position.rotation.x, msg->origin_position.rotation.y, msg->origin_position.rotation.z, msg->origin_position.rotation.w);

      T_body_world=T_world_body.Inverse(); 


    int64_t now = bot_timestamp_now();//msg->utime
    if(now-_last_state_msg_system_timestamp >= 100000)  // timestamps are in usec
    {
      // cout << now - _last_state_msg_system_timestamp << endl;
      _gl_robot->set_state(*msg);
      _last_state_msg_system_timestamp = now;//msg->utime;
      object_update_counter++;
      if(object_update_counter>0){ // tracking frequency
        updateGraspedObjectPosesViaFK();
        object_update_counter = 0;
      }
       
    }
   
      _last_robotstate_msg = (*msg);
      _parent_renderer->last_state_msg_timestamp = msg->utime;
      _parent_renderer->robot_name = msg->robot_name;
      //(*_parent_renderer->robot_name_ptr)  = msg->robot_name
    
  } // end handleMessage

//-------------------------------------------------------------------------------------        
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
    
      _gl_robot = shared_ptr<visualization_utils::GlKinematicBody>(new visualization_utils::GlKinematicBody(_urdf_xml_string));

      //remember that we've parsed the urdf already
      _urdf_parsed = true;
    }
 
  } // end urdf handler
//-------------------------------------------------------------------------------------        
  void RobotStateListener::updateGraspedObjectPosesViaFK(void)
  {
      typedef std::map<std::string, StickyHandStruc > sticky_hands_map_type_;
      sticky_hands_map_type_::iterator hand_it = _parent_renderer->sticky_hands.begin();
      while (hand_it!=_parent_renderer->sticky_hands.end()) 
      {
         std::string hand_name = std::string(hand_it->second.object_name);
         
         typedef map<string, OtdfInstanceStruc > object_instance_map_type_;
         object_instance_map_type_::iterator obj_it = _parent_renderer->instantiated_objects.find(string(hand_it->second.object_name));
         if((hand_it->second.is_melded)&&(obj_it->second.is_melded))
         {
            KDL::Frame T_world_palm, T_geometry_stickyhandbase,T_geometry_palm,T_palm_stickyhandbase,T_world_hand; 
            std::string ee_name;
            if(hand_it->second.hand_type==0)
               ee_name = "left_palm";
            else
               ee_name = "right_palm";
            T_geometry_stickyhandbase = hand_it->second._gl_hand->_T_world_body;
            hand_it->second._gl_hand->get_link_frame(ee_name,T_geometry_palm);         
            T_palm_stickyhandbase = T_geometry_palm.Inverse()*T_geometry_stickyhandbase;
            
            // Get the world frame location of the palm
            _gl_robot->get_link_frame(ee_name,T_world_palm);
            
            // transform to sticky hand base
            T_world_hand = T_world_palm*T_palm_stickyhandbase;
                 
            // where the associated geometry relative to the hand     
            KDL::Frame T_hand_geometry =  T_geometry_stickyhandbase.Inverse();
            
            // where is the associated geometry in the object frame            
            KDL::Frame T_world_geometry;
            obj_it->second._gl_object->get_link_geometry_frame(string(hand_it->second.geometry_name),T_world_geometry);  
            KDL::Frame T_world_object_old = obj_it->second._gl_object->_T_world_body;
            KDL::Frame T_geometry_object = T_world_geometry.Inverse()*T_world_object_old;
            
            if(!obj_it->second._gl_object->is_mateable())
            {
              // Working out where in the world should the object be from world space hand information from the robot state and 
              KDL::Frame T_world_object_new = T_world_hand*T_hand_geometry*T_geometry_object; 
              // cant set state directly interferes with affordance server
              //obj_it->second._gl_object->set_state(T_world_object_new,obj_it->second._gl_object->_current_jointpos); 
              update_object_pose_in_affstore("AFFORDANCE_TRACK",(obj_it->second.otdf_type),obj_it->second.uid,obj_it->second._otdf_instance, T_world_object_new,_parent_renderer);
            }
            else
            {
              double r,p,y;
              std::string mate_start_link =obj_it->second._gl_object->_mate_start_link;
              std::string mate_end_link = obj_it->second._gl_object->_mate_end_link;

              map<string, double> jointpos_in=obj_it->second._gl_object->_current_jointpos;
              typedef map<string,boost::shared_ptr<otdf::Joint> > joints_mapType;
              for (joints_mapType::iterator joint = obj_it->second._otdf_instance->joints_.begin();joint != obj_it->second._otdf_instance->joints_.end(); joint++)
              {     
                  double dof_pos = 0;
                  double vel;
                  string token  = "mate::";
                  string joint_name = joint->first;
                  size_t found = joint_name.find(token);  
                  if ((found!=std::string::npos)&&(joint->second->type!=(int) otdf::Joint::FIXED)) 
                  {
                      double pos, vel;
                      obj_it->second._otdf_instance->getJointState(joint->first,dof_pos,vel);
                      //if((joint->first=="mate::pitch")||(joint->first=="mate::roll")||(joint->first=="mate::yaw"))
                      //  cout <<  joint->first << " DOF changed to " << dof_pos << endl;
                  }// end if
               } // end for
              KDL::Frame T_world_mate_start,T_world_mate_end;
              obj_it->second._gl_object->get_link_frame(mate_start_link,T_world_mate_start);  
              obj_it->second._gl_object->get_link_frame(mate_end_link,T_world_mate_end); 

              // must compensate for the mate::end joint origin as we want the mate::yaw link frame basically  .
              // mate::end joint is not made fixed as KDL FK does not handle successive fixed joitns.
              //TODO: get_link_frame("mate::yaw") is returning zeros, could be of KDL fk not handling two fixed joints.           
              boost::shared_ptr<const otdf::Joint> joint;
              joint=obj_it->second._otdf_instance->getJoint("mate::end");
              KDL::Frame T_mateend_jointorigin;
              T_mateend_jointorigin.p[0]= joint->parent_to_joint_origin_transform.position.x;
              T_mateend_jointorigin.p[1]= joint->parent_to_joint_origin_transform.position.y;
              T_mateend_jointorigin.p[2]= joint->parent_to_joint_origin_transform.position.z;
              double qx,qy,qz,qw;       
              qx= joint->parent_to_joint_origin_transform.rotation.x;
              qy= joint->parent_to_joint_origin_transform.rotation.y;
              qz= joint->parent_to_joint_origin_transform.rotation.z;
              qw= joint->parent_to_joint_origin_transform.rotation.w;  
              T_mateend_jointorigin.M = KDL::Rotation::Quaternion(qx,qy,qz,qw);
              T_world_mate_end =T_world_mate_end*(T_mateend_jointorigin.Inverse());

              KDL::Frame T_mate_start_mate_end = (T_world_mate_start.Inverse())*(T_world_hand*T_hand_geometry*T_world_geometry.Inverse())*T_world_mate_end;
              update_mate_joints_in_affstore("AFFORDANCE_TRACK",(obj_it->second.otdf_type),obj_it->second.uid,obj_it->second._otdf_instance,T_mate_start_mate_end,_parent_renderer);
            }
         }
         hand_it++;
      }
  }

} //namespace renderer_robot_state


