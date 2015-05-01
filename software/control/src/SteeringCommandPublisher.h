#ifndef STEERING_COMMAND_PUBLISHER_H
#define STEERING_COMMAND_PUBLISHER_H

#include <string>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/driving_control_cmd_t.hpp"

#include "Joystick.h"

class SteeringCommandPublisher
{
public:
  SteeringCommandPublisher(std::string const & device_name, std::string const & channel_name);
  void publish();
  bool good() const;
private:
  drc::driving_control_cmd_t build_message(js_event const & jse) const;
  int8_t get_message_type(js_event const & jse) const;
  void set_message_value(js_event const & jse, const int8_t msg_type, drc::driving_control_cmd_t & msg) const;
  Joystick m_joystick;  
  lcm::LCM m_lcm;
  std::string m_channel_name;
};

#endif
