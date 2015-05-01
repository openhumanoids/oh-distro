#ifndef JOYSTICK_H
#define JOYSTICK_H
#include <string>

#define JOYSTICK_SUCCESS 1
#define JS_EVENT_AXIS 2

struct js_event {
	unsigned int time;    /* event timestamp in milliseconds */
	short value;          /* value */
	unsigned char type;   /* event type */
	unsigned char number; /* axis/button number */
};

class Joystick 
{
public:
  Joystick(std::string const & device_name);
  ~Joystick();
  int read_joystick_event(js_event & jse) const;
  bool is_open() const;
private:
	void open_joystick();
	void close_joystick();
	std::string m_device_name;
	int m_joystick_fd;
};

#endif