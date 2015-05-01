#include "GamepadPublisher.h"

int main(int argc, char *argv[])
{
  if (argc != 3) {
    printf("Usage: GamepadDriver <joystick-device> <lcm-channel>\n e.g. GamepadDriver /dev/input/js0 GAMEPAD_COMMAND\n");
    exit(1);
  }

  GamepadPublisher gamepad_publisher(argv[1], argv[2]);

  while (true) {
    gamepad_publisher.publish();
  }

  return 0;
}
