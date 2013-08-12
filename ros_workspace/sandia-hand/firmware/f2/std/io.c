#include "io.h"
#include "pins.h"

void io_led(bool on)
{
  if (on)
    PIO_Set(&pin_led);
  else
    PIO_Clear(&pin_led);
}

void io_led_toggle()
{
  static int io_led_prev_state = 0;
  if (io_led_prev_state)
    PIO_Set(&pin_led);
  else
    PIO_Clear(&pin_led);
  io_led_prev_state = !io_led_prev_state;
}

