// this is intended to be #included after a few #defines are set

#define MAKE_ID(NAME) ID_ ## NAME

#if   BL_USART_IDX == 0
  #define BL_USART    USART0
  #define BL_USART_ID ID_USART0
#elif BL_USART_IDX == 1
  #define BL_USART    USART1
  #define BL_USART_ID ID_USART1
#else
  #error unhandled BL_USART definition.
#endif

#ifdef BL_CHAIN_INIT
void BL_CHAIN_INIT ();
#endif

void bl_init()
{
  PMC->PMC_PCER0 = (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_PIOC);

  // set up the LED pin
  BL_LED_PIO->PIO_CODR = BL_LED_PIN;
  BL_LED_PIO->PIO_OER = BL_LED_PIN;
  BL_LED_PIO->PIO_PER = BL_LED_PIN;

  // set up the rs485 DE pin
  BL_RS485_DE_PIO->PIO_CODR = BL_RS485_DE_PIN;
  BL_RS485_DE_PIO->PIO_OER  = BL_RS485_DE_PIN;
  BL_RS485_DE_PIO->PIO_PER  = BL_RS485_DE_PIN;
  
  // set up the rs485 usart
  PMC->PMC_PCER0 = (1 << BL_USART_ID);
#if BL_USART_IDX == 0
  PIOA->PIO_PDR  = PIO_PA5A_RXD0 | PIO_PA6A_TXD0;
  PIOA->PIO_ABCDSR[0] &= ~(PIO_PA5A_RXD0 | PIO_PA6A_TXD0);
  PIOA->PIO_ABCDSR[1] &= ~(PIO_PA5A_RXD0 | PIO_PA6A_TXD0);
#elif BL_USART_IDX == 1
  PIOA->PIO_PDR  = PIO_PA21A_RXD1 | PIO_PA22A_TXD1;
  PIOA->PIO_ABCDSR[0] &= ~(PIO_PA21A_RXD1 | PIO_PA22A_TXD1);
  PIOA->PIO_ABCDSR[1] &= ~(PIO_PA21A_RXD1 | PIO_PA22A_TXD1);
#endif

  BL_USART->US_CR   = US_CR_RSTRX | US_CR_RSTTX |
                      US_CR_RXDIS | US_CR_TXDIS; // reset uart
#ifdef BL_RS485_MANCHESTER
  BL_USART->US_MR   = US_MR_CHRL_8_BIT | US_MR_PAR_NO |
                      US_MR_MAN | US_MR_OVER | US_MR_MODSYNC;
  USART0->US_BRGR = 64000000 / 2000000 / 16;
  BL_USART->US_MAN  = US_MAN_TX_PL(1) | US_MAN_TX_PP_ALL_ONE |
                      US_MAN_RX_PL(1) | US_MAN_RX_PP_ALL_ONE |
                      US_MAN_TX_MPOL | US_MAN_RX_MPOL |
                      US_MAN_DRIFT | US_MAN_STUCKTO1;
  BL_USART->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS; // disable DMA
#else
  BL_USART->US_MR   = US_MR_CHRL_8_BIT | US_MR_PAR_NO; // 8N1, normal mode
  BL_USART->US_BRGR = 64000000 / 2000000 / 16;
#endif
  BL_USART->US_CR   = US_CR_TXEN | US_CR_RXEN; // enable TX and RX

#ifdef BL_CHAIN_INIT
  (BL_CHAIN_INIT)();
#endif
}


