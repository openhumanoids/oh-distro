#include "led.h"
#include "common_sam3x/sam3x.h"

#define PORTA_LED_0  PIO_PA23
#define PORTA_LED_1  PIO_PA22

void led_init()
{
  PMC->PMC_PCER0 |= (1 << ID_PIOA);
  PIOA->PIO_PER = PIOA->PIO_OER = PIOA->PIO_CODR = PORTA_LED_0 | PORTA_LED_1;
}

void led_on(uint8_t led_idx)
{
  if (led_idx == 0)
    PIOA->PIO_SODR = PORTA_LED_0;
  else if (led_idx == 1)
    PIOA->PIO_SODR = PORTA_LED_1;
  else
    return;
}

void led_off(uint8_t led_idx)
{
  if (led_idx == 0)
    PIOA->PIO_CODR = PORTA_LED_0;
  else if (led_idx == 1)
    PIOA->PIO_CODR = PORTA_LED_1;
  else
    return;
}

void led_dance()
{
  if (PIOA->PIO_PDSR & PORTA_LED_0)
  {
    led_off(0);
    led_on(1);
  }
  else
  {
    led_on(0);
    led_off(1);
  }
}

