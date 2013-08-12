#include "bl_stubs.h"

const uint32_t __attribute__((section (".bl_hw_version"))) 
                             g_bl_hw_version = 0xbeef0041; // lower byte: 'A'
const int __attribute__((section (".rs485_addr")))    g_rs485_addr    =  10;

#define BL_LED_PIO           PIOA
#define BL_LED_PIN           PIO_PA31

#define BL_USART_IDX         0
#define BL_RS485_DE_PIO      PIOA
#define BL_RS485_DE_PIN      PIO_PA7
#define BL_CHAIN_INIT        bl_fmcb_init

#include "bl_init.c"
#include "bl_led.c"
#include "bl_uart.c"

void bl_fmcb_init()
{
  // drive the phalange power control line low
  PIOA->PIO_CODR = PIOA->PIO_OER = PIOA->PIO_PER = PIO_PA27;
}

