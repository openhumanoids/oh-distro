#include "sam3s/chip.h"
#include "pins.h"
#include "gpio.h"

void gpio_led(bool on)
{
  if (on)
    PIO_Set(&pin_led);
  else
    PIO_Clear(&pin_led);
}

