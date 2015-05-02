#ifndef GAMEPAD_PUBLISHER_H
#define GAMEPAD_PUBLISHER_H

#include <string>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/gamepad_cmd_t.hpp"
#include "Joystick.h"

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
