#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/driving_control_cmd_t.hpp"

#define JOYSTICK_DEVNAME "/dev/input/js0"
#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */
#define UNSUPPORTED_MSG_TYPE -1
#define STEERING_AXIS 0
#define ACCEL_AXIS 1
#define BRAKE_AXIS 2
#define MAX_STEERING_ANGLE 5.0*M_PI_2
#define THROTTLE_MULTIPLIER 0.5

static int joystick_fd = -1;

struct js_event {
	unsigned int time;	/* event timestamp in milliseconds */
	short value;   /* value */
	unsigned char type;     /* event type */
	unsigned char number;   /* axis/button number */
};

int open_joystick()
{
	joystick_fd = open(JOYSTICK_DEVNAME, O_RDONLY | O_NONBLOCK);
	if (joystick_fd < 0)
		return joystick_fd;

	return joystick_fd;
}

int read_joystick_event(struct js_event *jse)
{
	int bytes;

	bytes = read(joystick_fd, jse, sizeof(*jse)); 

	if (bytes == -1)
		return 0;

	if (bytes == sizeof(*jse))
		return 1;

	printf("Unexpected bytes from joystick:%d\n", bytes);

	return -1;
}

void close_joystick()
{
	close(joystick_fd);
}

int8_t getMessageType(js_event const & jse)
{
  int8_t msg_type = UNSUPPORTED_MSG_TYPE;

  if (jse.type == JS_EVENT_AXIS) {
    
    switch (jse.number) {
      case STEERING_AXIS:
        msg_type =  drc::driving_control_cmd_t::TYPE_DRIVE_DELTA_STEERING;
        break;
      case ACCEL_AXIS:
        msg_type = drc::driving_control_cmd_t::TYPE_DRIVE;
        break;
      case BRAKE_AXIS:
        msg_type =  drc::driving_control_cmd_t::TYPE_BRAKE;
        break;
    }
  } 

  return msg_type;
}

void setMessageValue(js_event const & jse, const int8_t msg_type, drc::driving_control_cmd_t & msg)
{
  static const double scaleFactor = static_cast<double>((1<<15) - 1);
  switch (msg_type) {
    case drc::driving_control_cmd_t::TYPE_DRIVE_DELTA_STEERING:
      msg.steering_angle = -jse.value / scaleFactor * MAX_STEERING_ANGLE;
      printf("steering: %lf\n", msg.steering_angle);
      break;
    case drc::driving_control_cmd_t::TYPE_DRIVE:
      msg.throttle_value = THROTTLE_MULTIPLIER*(-0.5*jse.value / scaleFactor + 0.5);
      printf("throttle: %lf\n", msg.throttle_value);
    break;
    case drc::driving_control_cmd_t::TYPE_BRAKE:
      msg.brake_value = -0.5*jse.value / scaleFactor + 0.5;
      printf("brake: %lf\n", msg.brake_value);
    break;
  }
}

void publish_command(js_event const & jse, lcm::LCM & lcm)
{
  drc::driving_control_cmd_t driving_control_cmd;
  int8_t msg_type = getMessageType(jse);
  
  if (msg_type != UNSUPPORTED_MSG_TYPE) {
    driving_control_cmd.utime = jse.time;
    driving_control_cmd.type = msg_type;
    setMessageValue(jse, msg_type, driving_control_cmd);
    lcm.publish("STEERING_COMMAND", &driving_control_cmd);
  } else {
    printf("Unpublished Event: time %8u, value %8hd, type: %3u, axis/button: %u\n", jse.time, jse.value, jse.type, jse.number);
  }
}

int main(int argc, char *argv[])
{
	int fd, rc;
	int done = 0;

	struct js_event jse;
	lcm::LCM lcm;
	if (!lcm.good()){
		printf("lcm was no good\n");
		return 1;
	}

	fd = open_joystick();
	
	if (fd < 0) {
		printf("cant open joystick: %s.\n", JOYSTICK_DEVNAME);
		exit(1);
	}

	while (!done) {
		rc = read_joystick_event(&jse);
		usleep(1000); 
		if (rc == 1) {
			publish_command(jse, lcm);
		} 
	}

	return 0;
}
