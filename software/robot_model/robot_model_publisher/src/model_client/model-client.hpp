#include <map>
#include <boost/shared_ptr.hpp>

#include "urdf/model.h"
#include <lcm/lcm.h>
#include "lcmtypes/drc_lcmtypes.h"
#include <fstream>

class ModelClient 
{
  public:
    ModelClient (lcm_t* lcm_, int keep_updated);
    ModelClient(lcm_t* lcm_, std::string model_channel_, int keep_updated_);
    
    // Get the model client directly from the specified file:
    // robot name isn't important
    ModelClient(std::string urdf_filename);

    ~ModelClient() {}
    
    void doModelClient();

    static void drc_robot_urdf_handler_aux(const lcm_recv_buf_t* rbuf, const char* channel, 
                                            const drc_robot_urdf_t* msg, void* user_data){
      ((ModelClient *) user_data)->drc_robot_urdf_handler(channel, msg);
    }
    void drc_robot_urdf_handler(const char* channel, const drc_robot_urdf_t *msg);      
    
    // Returns false if urdf isn't an XML string:
    bool readURDFFromFile(std::string);
    
    void parseURDFString();
    // determine which hands are connected to the robot and store that info
    // this information is higher level than just the string as it assumes an lcmtype
    void setHandConfiguration();

    std::string getURDFString(){ return urdf_xml_string_; }
    std::string getRobotName(){ return robot_name_; }
    std::vector<std::string> getJointNames(){ return joint_names_; };
    int8_t getLeftHand(){ return left_hand_; };
    int8_t getRightHand(){ return right_hand_; };

  private:
    lcm_t* lcm_;
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    bool urdf_parsed_;
    std::string model_channel_;
    std::string robot_name_;
    std::string urdf_xml_string_;
    std::vector<std::string> joint_names_;    
    std::map<std::string, boost::shared_ptr<urdf::Link> > links_map_;
    int keep_updated_;
    
    int8_t left_hand_, right_hand_;
};
