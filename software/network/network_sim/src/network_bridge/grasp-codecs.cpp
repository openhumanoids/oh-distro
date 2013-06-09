#include "grasp-codecs.h"
#include "robot-state-codecs.h"

GraspCodec::GraspCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get())
{
    dccl_->validate<drc::MinimalGrasp>();
}

bool GraspCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;
    
    drc::desired_grasp_state_t present_grasp;
    if(present_grasp.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;

    drc::MinimalGrasp dccl_grasp;

    dccl_grasp.set_utime(present_grasp.utime);
        
    dccl_grasp.set_unique_id(present_grasp.unique_id);

    dccl_grasp.set_grasp_type(present_grasp.grasp_type);
    dccl_grasp.set_power_grasp(present_grasp.power_grasp);

    if(!RobotStateCodec::to_minimal_position3d(present_grasp.l_hand_pose, dccl_grasp.mutable_l_hand_pose()))
        return false;

    if(!RobotStateCodec::to_minimal_position3d(present_grasp.r_hand_pose, dccl_grasp.mutable_r_hand_pose()))
        return false;
        

    int l_hand_offset = RobotStateCodec::joint_names_to_order_["left_f0_j0"];
    int r_hand_offset = RobotStateCodec::joint_names_to_order_["right_f0_j0"];
    

    if(!RobotStateCodec::to_minimal_joint_pos(present_grasp.l_joint_name,
                                              present_grasp.l_joint_position,
                                              dccl_grasp.mutable_l_joint_position(),
                                              l_hand_offset))
        return false;

    if(!RobotStateCodec::to_minimal_joint_pos(present_grasp.r_joint_name,
                                              present_grasp.r_joint_position,
                                              dccl_grasp.mutable_r_joint_position(),
                                              r_hand_offset))
        return false;

    dccl_grasp.set_optimization_status(present_grasp.optimization_status);
    
    
    if(glog.is(VERBOSE))
    {
        google::protobuf::TextFormat::Printer printer;
        printer.SetUseShortRepeatedPrimitives(true);
        std::string dccl_grasp_debug;
        printer.PrintToString(dccl_grasp, &dccl_grasp_debug);
        glog << "MinimalGrasp: " << dccl_grasp_debug << std::endl;
    }
    
    std::string encoded;
    dccl_->encode(&encoded, dccl_grasp);
    transmit_data->resize(encoded.size());
    std::copy(encoded.begin(), encoded.end(), transmit_data->begin());
    return true;
}

      
bool GraspCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;

    std::string encoded(transmit_data.begin(), transmit_data.end());
    drc::MinimalGrasp dccl_grasp;

    dccl_->decode(encoded, &dccl_grasp);
    
    if(glog.is(VERBOSE))
    {
        google::protobuf::TextFormat::Printer printer;
        printer.SetUseShortRepeatedPrimitives(true);
        std::string dccl_grasp_debug;
        printer.PrintToString(dccl_grasp, &dccl_grasp_debug);
        glog << "MinimalGrasp: " << dccl_grasp_debug << std::endl;
    }
    
    drc::desired_grasp_state_t present_grasp;

    present_grasp.utime = dccl_grasp.utime();
    present_grasp.robot_name = "atlas";

    present_grasp.object_name = "";
    present_grasp.geometry_name = "";
    

    present_grasp.unique_id = dccl_grasp.unique_id();
    present_grasp.grasp_type = dccl_grasp.grasp_type();
    present_grasp.power_grasp = dccl_grasp.power_grasp();

    
    if(!RobotStateCodec::from_minimal_position3d(&present_grasp.l_hand_pose, dccl_grasp.l_hand_pose()))
        return false;

    if(!RobotStateCodec::from_minimal_position3d(&present_grasp.r_hand_pose, dccl_grasp.r_hand_pose()))
        return false;
    
    int l_hand_offset = RobotStateCodec::joint_names_to_order_["left_f0_j0"];
    int r_hand_offset = RobotStateCodec::joint_names_to_order_["right_f0_j0"];

    present_grasp.num_l_joints = dccl_grasp.l_joint_position_size();
    if(!RobotStateCodec::from_minimal_joint_pos(&present_grasp.l_joint_name,
                                                &present_grasp.l_joint_position,
                                                dccl_grasp.l_joint_position(),
                                                l_hand_offset))
        return false;

    present_grasp.num_r_joints = dccl_grasp.r_joint_position_size();    
    if(!RobotStateCodec::from_minimal_joint_pos(&present_grasp.r_joint_name,
                                                &present_grasp.r_joint_position,
                                                dccl_grasp.r_joint_position(),
                                                r_hand_offset))
        return false;
    

    present_grasp.optimization_status = dccl_grasp.optimization_status();
    
    lcm_data->resize(present_grasp.getEncodedSize());
    present_grasp.encode(&(*lcm_data)[0], 0, lcm_data->size());

    
    return true;
          
}      
