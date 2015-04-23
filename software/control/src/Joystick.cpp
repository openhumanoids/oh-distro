#include "Joystick.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;

Joystick::Joystick(string const & device_name) 
: m_device_name(device_name), m_joystick_fd(-1)
{
  open_joystick();
}

Joystick::~Joystick()
{
  close_joystick(); 
}

void Joystick::open_joystick()
{
  m_joystick_fd = open(m_device_name.c_str(), O_RDONLY | O_NONBLOCK);
}

void Joystick::close_joystick()
{
  if (is_open()) {
    close(m_joystick_fd);
  }
}

bool Joystick::is_open() const
{
  return m_joystick_fd >= 0;
}

int Joystick::read_joystick_event(js_event & jse) const 
{
  int bytes;

  bytes = read(m_joystick_fd, &jse, sizeof(jse)); 

  if (bytes == -1)
   return 0;

  if (bytes == sizeof(jse))
   return 1;

  printf("Unexpected bytes from joystick:%d\n", bytes);

  return -1;  
}
