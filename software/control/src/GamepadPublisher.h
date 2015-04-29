#ifndef GAMEPAD_PUBLISHER_H
#define GAMEPAD_PUBLISHER_H

#include <string>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/gamepad_cmd_t.hpp"

#include "Joystick.h"

#define GAMEPAD_DISCRETE 1
#define GAMEPAD_CONTINUOUS 2

#define GAMEPAD_BUTTON_A 0
#define GAMEPAD_BUTTON_B 1
#define GAMEPAD_BUTTON_X 2
#define GAMEPAD_BUTTON_Y 3
#define GAMEPAD_BUTTON_START 7
#define GAMEPAD_BUTTON_BACK 6
#define GAMEPAD_BUTTON_LB 4
#define GAMEPAD_BUTTON_RB 5
#define GAMEPAD_BUTTON_JL 9
#define GAMEPAD_BUTTON_JR 10
#define GAMEPAD_RIGHT_X 3
#define GAMEPAD_RIGHT_Y 4
#define GAMEPAD_LEFT_X 0
#define GAMEPAD_LEFT_Y 1
#define GAMEPAD_DPAD_X 6
#define GAMEPAD_DPAD_Y 7
#define GAMEPAD_TRIGGER_L 2
#define GAMEPAD_TRIGGER_R 5

class GamepadPublisher
{
public:
  GamepadPublisher(std::string const & device_name, std::string const & channel_name);
  void publish();
private:
  drc::gamepad_cmd_t build_message(js_event const & jse) const;
  void process_discrete_event(js_event const & jse, drc::gamepad_cmd_t & gamepad_cmd) const; //for up/down buttons
  void process_continuous_event(js_event const & jse, drc::gamepad_cmd_t & gamepad_cmd) const; //for joysticks and pedals
  Joystick m_joystick;  
  lcm::LCM m_lcm;
  std::string m_channel_name;
};

#endif
