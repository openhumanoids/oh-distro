// must be #included after bl_init.c

int bl_uart_byte_ready()
{
  return (BL_USART->US_CSR & US_CSR_RXRDY);
}

uint8_t bl_uart_get_byte()
{
  return (uint8_t)BL_USART->US_RHR;
}

void bl_uart_tx(const uint8_t *data, const uint16_t data_len)
{
  BL_USART->US_CR |= US_CR_RXDIS;
  BL_RS485_DE_PIO->PIO_SODR = BL_RS485_DE_PIN;
  while ((BL_USART->US_CSR & US_CSR_TXEMPTY) == 0) { }
  for (volatile uint32_t i = 0; i < (7+data_len); i++)
  {
    BL_USART->US_THR = data[i];
    while ((BL_USART->US_CSR & US_CSR_TXEMPTY) == 0) { }
  } 
  BL_RS485_DE_PIO->PIO_CODR = BL_RS485_DE_PIN; // release the rs485 bus
  BL_USART->US_CR &= ~US_CR_RXDIS;
  BL_USART->US_CR |=  US_CR_RXEN;
}

