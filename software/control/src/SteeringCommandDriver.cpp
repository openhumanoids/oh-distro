#include "SteeringCommandPublisher.h"

int main(int argc, char *argv[])
{
  if (argc != 3) {
    printf("Usage: SteeringCommandDriver <joystick-device> <lcm-channel>\n e.g. SteeringCommandDriver /dev/input/js0 STEERING_COMMAND\n");
    exit(1);
  }

  SteeringCommandPublisher steering_publisher(argv[1], argv[2]);

  while (true) {
    steering_publisher.publish();
  }

  return 0;
}
