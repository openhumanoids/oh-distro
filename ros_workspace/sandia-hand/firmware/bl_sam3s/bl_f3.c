#include "bl_stubs.h"

const uint32_t __attribute__((section (".bl_hw_version"))) 
                             g_bl_hw_version = 0xbeef0041; // lower byte: 'A'
const int __attribute__((section (".rs485_addr")))    g_rs485_addr    =   2;

#define BL_LED_PIO           PIOA
#define BL_LED_PIN           PIO_PA30

#define BL_USART_IDX         0
#define BL_RS485_DE_PIO      PIOC
#define BL_RS485_DE_PIN      PIO_PC9
#define BL_RS485_MANCHESTER

#include "bl_init.c"
#include "bl_led.c"
#include "bl_uart.c"

