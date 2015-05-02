#include "SteeringCommandPublisher.h"
#include <math.h>

#define UNSUPPORTED_MSG_TYPE -1
#define STEERING_AXIS 0
#define ACCEL_AXIS 1
#define BRAKE_AXIS 2
#define MAX_STEERING_ANGLE 5.0*M_PI_2
#define THROTTLE_MULTIPLIER 0.5

using namespace std;
using namespace drc;

int8_t SteeringCommandPublisher::get_message_type(js_event const & jse) const
{
  int8_t msg_type = UNSUPPORTED_MSG_TYPE;
  
  if (jse.type == JS_EVENT_AXIS) {
    
    switch (jse.number) {
      case STEERING_AXIS:
        msg_type =  driving_control_cmd_t::TYPE_DRIVE_DELTA_STEERING;
        break;
      case ACCEL_AXIS:
        msg_type = driving_control_cmd_t::TYPE_DRIVE;
        break;
      case BRAKE_AXIS:
        msg_type = driving_control_cmd_t::TYPE_BRAKE;
        break;
    }
  } 

  return msg_type;
}

void SteeringCommandPublisher::set_message_value(js_event const & jse, const int8_t msg_type, driving_control_cmd_t & msg) const
{
  static const double scaleFactor = static_cast<double>((1<<15) - 1);
  switch (msg_type) {
    case driving_control_cmd_t::TYPE_DRIVE_DELTA_STEERING:
      msg.steering_angle = -jse.value / scaleFactor * MAX_STEERING_ANGLE;
      printf("steering: %lf\n", msg.steering_angle);
      break;
    case driving_control_cmd_t::TYPE_DRIVE:
      msg.throttle_value = THROTTLE_MULTIPLIER*(-0.5*jse.value / scaleFactor + 0.5);
      printf("throttle: %lf\n", msg.throttle_value);
    break;
    case driving_control_cmd_t::TYPE_BRAKE:
      msg.brake_value = -0.5*jse.value / scaleFactor + 0.5;
      printf("brake: %lf\n", msg.brake_value);
    break;
  }
}


SteeringCommandPublisher::SteeringCommandPublisher(string const & device_name, string const & channel_name)
: m_channel_name(channel_name), m_joystick(device_name) { }

driving_control_cmd_t SteeringCommandPublisher::build_message(js_event const & jse) const
{
  driving_control_cmd_t driving_control_cmd;
  int8_t msg_type = get_message_type(jse);
  if (msg_type != UNSUPPORTED_MSG_TYPE) {
    driving_control_cmd.utime = jse.time;
    driving_control_cmd.type = msg_type;
    set_message_value(jse, msg_type, driving_control_cmd);
  } else {
    printf("Unpublished Event: time %8u, value %8hd, type: %3u, axis/button: %u\n", jse.time, jse.value, jse.type, jse.number);
  }

  return driving_control_cmd;
}

void SteeringCommandPublisher::publish()
{
  js_event jse;
  int rc = m_joystick.read_joystick_event(jse);
  if (rc == JOYSTICK_SUCCESS) {
    driving_control_cmd_t driving_control_cmd = build_message(jse);
    if (driving_control_cmd.type != UNSUPPORTED_MSG_TYPE) {
      m_lcm.publish(m_channel_name, &driving_control_cmd);
    }
  }
}

bool SteeringCommandPublisher::good() const
{
  return m_lcm.good() && m_joystick.is_open();
}