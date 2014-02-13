// NBNBNBNBNBNBNBNBNBNBNBNBNBNBNBNB
// Any subscriptions/construction with Model Client must
// be before typical user subscriptions - if the same LCM object
// is used. Otherwise it won't be received before handling them
// NBNBNBNBNBNBNBNBNBNBNBNBNBNBNBNB
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
  while ((_timestamp_now() - utime_start) < 2.5e6) {
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
      
  file_read_success_ = false; // file wasn't read, book keeping
  doModelClient();  
}

ModelClient::ModelClient(std::string urdf_filename){
  // Received robot urdf string. Store it internally and get all available joints.
  file_read_success_ = readURDFFromFile(urdf_filename);
  if (file_read_success_){
    std::cout<< "Read urdf_xml_string of robot [" 
        << robot_name_ << "] from file, storing it internally as a param" << std::endl;
    parseURDFString();  
  }else{
    std::cout<< urdf_filename << " could not be read. exiting" << std::endl;
    exit(-1);
  }
}



void ModelClient::drc_robot_urdf_handler(const char* channel, const drc_robot_urdf_t *msg){
  
  // Received robot urdf string. Store it internally and get all available joints.
  robot_name_      = msg->robot_name;
  urdf_xml_string_ = msg->urdf_xml_string;
  std::cout<< "Received urdf_xml_string of robot [" 
      << msg->robot_name << "], storing it internally as a param" << std::endl;
  parseURDFString();
}

void ModelClient::parseURDFString(){      
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
  
  std::cout<< "Number of Joints: " << joint_names_.size() <<std::endl;  
  setHandConfiguration();
  
  urdf_parsed_  = true;
  
}



bool ModelClient::readURDFFromFile(std::string urdf_file){

  // get the entire file
  std::string xml_string;
  std::fstream xml_file(urdf_file.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);     
      xml_string += (line + "\n");
    }
    xml_file.close();
    std::cout << "File ["<< urdf_file << "]  parsed successfully.\n";    
  }
  else
  {
    std::cout << "ERROR: Could not open file ["<< urdf_file << "] for parsing.\n";
    return false;
  }
  
  // Get a urdf Model from the xml string and get all the joint names.
  urdf::Model robot_model; 
  if (!robot_model.initString( xml_string ))
  {
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
  }

  // Set the member variable:
  urdf_xml_string_ = xml_string;
  robot_name_ = robot_model.getName();
}


void ModelClient::setHandConfiguration(){

  if(find(joint_names_.begin(), joint_names_.end(), "left_f0_j0" ) != joint_names_.end()){
    std::cout << "Robot fitted with left Sandia hand\n";
    left_hand_ = DRC_ROBOT_URDF_T_LEFT_SANDIA; //drc::robot_urdf_t::LEFT_SANDIA;
  }else if(find(joint_names_.begin(), joint_names_.end(), "left_finger[0]/joint_base" ) != joint_names_.end()){
    std::cout << "Robot fitted with left iRobot hand\n";
    left_hand_ = DRC_ROBOT_URDF_T_LEFT_IROBOT; //drc::robot_urdf_t::LEFT_IROBOT;
  }else if(find(joint_names_.begin(), joint_names_.end(), "left_finger_1_joint_1" ) != joint_names_.end()){
    std::cout << "Robot fitted with left Robotiq hand\n";
    left_hand_ = DRC_ROBOT_URDF_T_LEFT_ROBOTIQ; //drc::robot_urdf_t::LEFT_ROBOTIQ;
  }else{
    std::cout << "Robot has no left hand\n"; 
    left_hand_ = DRC_ROBOT_URDF_T_LEFT_NONE; //drc::robot_urdf_t::LEFT_NONE;
  }

  if(find(joint_names_.begin(), joint_names_.end(), "right_f0_j0" ) != joint_names_.end()){
    std::cout << "Robot fitted with right Sandia hand\n";
    right_hand_ = DRC_ROBOT_URDF_T_RIGHT_SANDIA;//drc::robot_urdf_t::RIGHT_SANDIA;
  }else if(find(joint_names_.begin(), joint_names_.end(), "right_finger[0]/joint_base" ) != joint_names_.end()){
    std::cout << "Robot fitted with right iRobot hand\n";
    right_hand_ = DRC_ROBOT_URDF_T_RIGHT_IROBOT;//drc::robot_urdf_t::RIGHT_IROBOT;
  }else if(find(joint_names_.begin(), joint_names_.end(), "right_finger_1_joint_1" ) != joint_names_.end()){
    std::cout << "Robot fitted with right Robotiq hand\n";
    right_hand_ = DRC_ROBOT_URDF_T_RIGHT_ROBOTIQ;//drc::robot_urdf_t::RIGHT_ROBOTIQ;
  }else{
    std::cout << "Robot has no right hand\n"; 
    right_hand_ = DRC_ROBOT_URDF_T_RIGHT_NONE;//drc::robot_urdf_t::RIGHT_NONE;
  }
}
