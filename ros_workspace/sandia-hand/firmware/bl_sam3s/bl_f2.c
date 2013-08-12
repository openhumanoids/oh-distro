#include "sam3s/chip.h"
#include "bl_stubs.h"

const uint32_t __attribute__((section (".bl_hw_version"))) 
                             g_bl_hw_version = 0xbeef0041; // lower byte: 'A'
const int __attribute__((section (".rs485_addr")))    g_rs485_addr    =   1;

#define BL_LED_PIO           PIOA
#define BL_LED_PIN           PIO_PA22

#define BL_USART_IDX         0
#define BL_RS485_DE_PIO      PIOC
#define BL_RS485_DE_PIN      PIO_PC9
#define BL_RS485_MANCHESTER

#include "bl_led.c"
#include "bl_init.c"

int bl_uart_byte_ready()
{
  return (USART0->US_CSR & US_CSR_RXRDY);
}

uint8_t bl_uart_get_byte()
{
  return (uint8_t)USART0->US_RHR;
}

void bl_uart_tx(const uint8_t *data, const uint16_t data_len)
{
  USART0->US_CR |= US_CR_RXDIS;
  PIOC->PIO_SODR = PIO_PC9;
  while ((USART0->US_CSR & US_CSR_TXEMPTY) == 0) { }
  for (volatile uint32_t i = 0; i < (7+data_len); i++)
  {
    USART0->US_THR = data[i];
    while ((USART0->US_CSR & US_CSR_TXEMPTY) == 0) { }
  } 
  PIOC->PIO_CODR = PIO_PC9; // release the rs485 bus
  USART0->US_CR &= ~US_CR_RXDIS;
  USART0->US_CR |= US_CR_RXEN;
}

