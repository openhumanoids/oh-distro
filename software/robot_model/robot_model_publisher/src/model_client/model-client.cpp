// file: robot_model_listener.cpp
// test code for a robot model subscriber.

#include <stdio.h>
#include <sys/time.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include "model-client.hpp"


/**
 * lcm_sleep:
 * @lcm: The lcm_t object.
 * @sleeptime max time to wait in seconds
 *
 *  Waits for up to @sleeptime seconds for an LCM message to arrive.
 *  It handles the first message if one arrives.
 *
 */
static inline void lcm_sleep(lcm_t * lcm, double sleeptime)
{ //
  int lcm_fileno = lcm_get_fileno(lcm);

  fd_set rfds;
  int retval;
  FD_ZERO(&rfds);
  FD_SET(lcm_fileno, &rfds);
  struct timeval tv;
  tv.tv_sec = (int) sleeptime;
  tv.tv_usec = (int) ((sleeptime - tv.tv_sec) * 1.0e6);
  retval = select(lcm_fileno + 1, &rfds, NULL, NULL, &tv);
  if (retval == -1) {
    fprintf(stderr, "bot_lcm_poll: select() failed!\n");
    return;
  }
  if (retval) {
    if (FD_ISSET(lcm_fileno, &rfds)) {
      lcm_handle(lcm);
    }
  }
}

static inline int64_t _timestamp_now()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


void ModelClient::doModelClient(){
  
  drc_robot_urdf_t_subscription_t * sub = drc_robot_urdf_t_subscribe(lcm_, model_channel_.c_str(), drc_robot_urdf_handler_aux, this);
  
  //TODO: is there a way to be sure nothing else is subscribed???
  int64_t utime_start = _timestamp_now();
  int64_t last_print_utime = -1;
  while ((_timestamp_now() - utime_start) < 1.5e6) {
    //bot_param_request_t req;
    //req.utime = _timestamp_now();
    //bot_param_request_t_publish(lcm, request_channel, &req);

    lcm_sleep(lcm_, .25);
    if (urdf_parsed_)
      break;
    int64_t now = _timestamp_now();
    if (now - utime_start > 5e5) {
      if (last_print_utime < 0) {
        fprintf(stderr, "ModelClient waiting to get model from robot_model_publisher...");
        last_print_utime = now;
      }
      else if (now - last_print_utime > 5e5) {
        fprintf(stderr, ".");
        last_print_utime = now;
      }
    }
  }


  if (last_print_utime > 0) {
    fprintf(stderr, "\n");
  }
  if (!urdf_parsed_) {
    fprintf(stderr,
        "WARNING: ModelClient could not get model from the robot_model_publisher!\n Did you forget to start one?\n");
    exit(-1);
    return;
  }

  if (!keep_updated_) {
    //unsubscribe from urdf messages
    drc_robot_urdf_t_unsubscribe(lcm_, sub);
  //  param->server_id = -1;
  }
}


ModelClient::ModelClient(lcm_t* lcm_, int keep_updated_):
    urdf_parsed_(false),lcm_(lcm_), keep_updated_(keep_updated_){
  model_channel_ = "ROBOT_MODEL";
  doModelClient(  );
}


// TODO implement update
ModelClient::ModelClient(lcm_t* lcm_, std::string model_channel_, int keep_updated_):
    urdf_parsed_(false),lcm_(lcm_),
    model_channel_(model_channel_), keep_updated_(keep_updated_){
  doModelClient();
  
}


void ModelClient::drc_robot_urdf_handler(const char* channel, const drc_robot_urdf_t *msg){
  
  // Received robot urdf string. Store it internally and get all available joints.
  robot_name_      = msg->robot_name;
  urdf_xml_string_ = msg->urdf_xml_string;
  std::cout<< "\nReceived urdf_xml_string of robot [" 
      << msg->robot_name << "], storing it internally as a param" << std::endl;

  // Get a urdf Model from the xml string and get all the joint names.
  urdf::Model robot_model; 
  if (!robot_model.initString( urdf_xml_string_ ))
  {
    std::cerr << "ERROR: Could not generate robot model" << std::endl;
  }

  typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joints_mapType;
  for( joints_mapType::const_iterator it = robot_model.joints_.begin(); it!=robot_model.joints_.end(); it++)
  {
    if(it->second->type!=6) // All joints that not of the type FIXED.
      joint_names_.push_back(it->first);
  }
  links_map_ =  robot_model.links_;

  //---------parse the tree and stop listening for urdf messages

  // Parse KDL tree
//  KDL::Tree tree;
//  if (!kdl_parser::treeFromString(urdf_xml_string_,tree))
//  {
//    std::cerr << "ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
//    return;
//  }

  //
//  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

  std::cout<< "Number of Joints: " << joint_names_.size() <<std::endl;  
  
  
  urdf_parsed_  = true;
}

