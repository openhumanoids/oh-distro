// Copyright 2016 Wolfgang Merkt

/**
 * Translates robotiqhand::command_t LCM messages to the DESIRED_HAND_ANGLES message expected by the
 * JointPositionGoalController for the hand.
 *
 * Currently supports only:
 *   - emergency_release
 *   - position mode (0-100% closed)
 */
#include <cstdlib>
#include <sys/time.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <array>
#include <map>
#include <vector>
#include <memory>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/robotiqhand/command_t.hpp"
#include "lcmtypes/bot_core/joint_angles_t.hpp"

int64_t bot_timestamp_now() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return static_cast<int64_t>(tv.tv_sec * 1e6 + tv.tv_usec);
}

inline double get_position_percentage_from_command_range(int16_t value) {
    return std::max(0.0, std::min(1.0, static_cast<double>(value / 254)));
}

enum Side {
    LEFT,
    RIGHT
};
std::map<Side, std::string> side_names_;
std::vector<std::string> hand_actuator_names_;
std::map<std::string, std::string> canonical_name_map_;

enum PresetHandPose {
    EMERGENCY_RELEASE,
    OPEN,
    CLOSED
};

typedef std::array<double, 6> HandPose;
// double thumbRoll;
// double thumbPitch1;
// double thumbPitch2;
// double indexFinger;
// double middleFinger;
// double pinky;

std::map<PresetHandPose, HandPose> preset_hand_poses_;
std::map<std::string, unsigned int> hand_pose_map_;

class LCM2ROSControl {
public:
    explicit LCM2ROSControl(std::shared_ptr<lcm::LCM> &lcm_in);

    ~LCM2ROSControl() { }

private:
    std::shared_ptr<lcm::LCM> lcm_;

    bot_core::joint_angles_t getHandControlMessage(Side side, HandPose pose);

    void leftHandCommandHandler(const lcm::ReceiveBuffer *rbuf,
                                const std::string &channel,
                                const robotiqhand::command_t *msg);

    void rightHandCommandHandler(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &channel,
                                 const robotiqhand::command_t *msg);

    void handCommandHandler(Side side, const robotiqhand::command_t *msg);
};

LCM2ROSControl::LCM2ROSControl(std::shared_ptr<lcm::LCM> &lcm_in)
        : lcm_(lcm_in) {
    lcm_->subscribe("ROBOTIQ_LEFT_COMMAND", &LCM2ROSControl::leftHandCommandHandler,
                    this);
    lcm_->subscribe("ROBOTIQ_RIGHT_COMMAND", &LCM2ROSControl::rightHandCommandHandler,
                    this);
}

void LCM2ROSControl::handCommandHandler(Side side, const robotiqhand::command_t *msg) {
    std::cout << "Received " << side_names_[side] << " hand command" << std::endl;

    bot_core::joint_angles_t hand_control_msg;

    if (msg->emergency_release == 1) {  // Check whether to perform an emergency release
        HandPose &pose = preset_hand_poses_[PresetHandPose::EMERGENCY_RELEASE];
        hand_control_msg = getHandControlMessage(Side::LEFT, pose);

        std::cerr << "Emergency release activated" << std::endl;

        lcm_->publish("DESIRED_HAND_ANGLES", &hand_control_msg);
        return;
    }

    // Hand Control Modes
    //
    // -1 - ignore (use previous mode)
    //  0 - basic (normal)
    //  1 - pinch
    //  2 - wide
    //  3 - scissor
    if (msg->do_move == 1) {
        if (msg->mode == 0) {  // Basic Position Mode (between 0 and 100% closed)
            HandPose &pose = preset_hand_poses_[PresetHandPose::CLOSED];

            double percentage_closed = get_position_percentage_from_command_range(msg->position);
            for (unsigned int i = 0; i < pose.size(); i++)
                pose[i] = pose[i] * percentage_closed;

            hand_control_msg = getHandControlMessage(Side::LEFT, pose);
        } else {
            std::cerr << "Mode " << msg->mode << " not implemented yet" << std::endl;
        }
    }


    // Individual finger control only allows 3 positions at the moment
    // static_cast<bool>(msg->ifc);
    // msg->positionA;
    // msg->positionB;
    // msg->positionC;

    // Individual scissor control
    // static_cast<bool>(msg->isc);
    // msg->positionS;

    // Publish to JointPositionGoalController
    hand_control_msg.utime = msg->utime;
    lcm_->publish("DESIRED_HAND_ANGLES", &hand_control_msg);
}

void LCM2ROSControl::rightHandCommandHandler(const lcm::ReceiveBuffer *rbuf,
                                             const std::string &channel,
                                             const robotiqhand::command_t *msg) {
    handCommandHandler(Side::RIGHT, msg);
}

void LCM2ROSControl::leftHandCommandHandler(const lcm::ReceiveBuffer *rbuf,
                                            const std::string &channel,
                                            const robotiqhand::command_t *msg) {
    handCommandHandler(Side::LEFT, msg);
}

bot_core::joint_angles_t LCM2ROSControl::getHandControlMessage(Side side, HandPose pose) {
    bot_core::joint_angles_t msg = bot_core::joint_angles_t();

    msg.utime = bot_timestamp_now();
    msg.robot_name = "Valkyrie";
    msg.num_joints = static_cast<int32_t>(hand_actuator_names_.size());
    msg.joint_name.resize(static_cast<int32_t>(hand_actuator_names_.size()));
    msg.joint_position.resize(static_cast<int32_t>(hand_actuator_names_.size()));

    unsigned int i = 0;
    for (auto &actuator : hand_actuator_names_) {
        msg.joint_name[i] = side_names_[side] + actuator;
        msg.joint_position[i] = pose[hand_pose_map_[canonical_name_map_[actuator]]];

        ++i;
    }

    return msg;
}

int main(int argc, char **argv) {
    // hand_actuator_names_ contains the names of all actuators in each hand without the left/right prefix
    hand_actuator_names_.push_back("ThumbMotorRoll");
    hand_actuator_names_.push_back("ThumbMotorPitch1");
    hand_actuator_names_.push_back("ThumbMotorPitch2");
    hand_actuator_names_.push_back("IndexFingerMotorPitch1");
    hand_actuator_names_.push_back("MiddleFingerMotorPitch1");
    hand_actuator_names_.push_back("PinkyMotorPitch1");

    side_names_.insert(std::make_pair(Side::LEFT, "left"));
    side_names_.insert(std::make_pair(Side::RIGHT, "right"));

    // canonical_name_map_ maps between the actuator name without the left/right prefix and a canonical name for the finger
    canonical_name_map_.insert(std::make_pair("ThumbMotorRoll", "thumbRoll"));
    canonical_name_map_.insert(std::make_pair("ThumbMotorPitch1", "thumbPitch1"));
    canonical_name_map_.insert(std::make_pair("ThumbMotorPitch2", "thumbPitch2"));
    canonical_name_map_.insert(std::make_pair("IndexFingerMotorPitch1", "indexFinger"));
    canonical_name_map_.insert(std::make_pair("MiddleFingerMotorPitch1", "middleFinger"));
    canonical_name_map_.insert(std::make_pair("PinkyMotorPitch1", "pinky"));

    // hand_pose_map_ maps between a canonical name and the array index in the HandPose data structure
    hand_pose_map_.insert(std::make_pair("thumbRoll", 0));
    hand_pose_map_.insert(std::make_pair("thumbPitch1", 1));
    hand_pose_map_.insert(std::make_pair("thumbPitch2", 2));
    hand_pose_map_.insert(std::make_pair("indexFinger", 3));
    hand_pose_map_.insert(std::make_pair("middleFinger", 4));
    hand_pose_map_.insert(std::make_pair("pinky", 5));

    // Some preset poses
    HandPose emergency_release_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    preset_hand_poses_.insert(std::make_pair(PresetHandPose::EMERGENCY_RELEASE, emergency_release_pose));

    HandPose open_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    preset_hand_poses_.insert(std::make_pair(PresetHandPose::OPEN, open_pose));

    HandPose closed_pose = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    preset_hand_poses_.insert(std::make_pair(PresetHandPose::CLOSED, closed_pose));

    std::shared_ptr<lcm::LCM> lcm = std::shared_ptr<lcm::LCM>(new lcm::LCM);
    if (!lcm->good())
        std::cerr << "ERROR: lcm is not good()" << std::endl;

    LCM2ROSControl handlerObject(lcm);
    std::cout << "LCM2ROSControl Valkyrie Hand Translator Ready" << std::endl;

    while (0 == lcm->handle()) { }

    return 0;
}
