//char bl_flash_page[512] __attribute__ ((section("bootloader_bss"))) = {0};
//char bl_stack[1000] __attribute__ ((section("bootloader_stack"))) = {0};
#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"

#define PIN_A_LED       31
#define PIN_A_RS485_RO  5
#define PIN_A_RS485_DI  6
#define PIN_A_RS485_DE  7
#define PIN_A_RS485_RE  8

#define MAX_BL_RX_LEN 1024
#define MAX_BL_TX_LEN 1024
#define BL_AUTOBOOT_COUNT 2000000

// phalage power enable line: PA9. need to drive it low.
#define PIN_A_PHALANGE_POWER 27

extern uint32_t _bl_sfixed;
extern uint32_t _bl_szero;
extern uint32_t _bl_ezero;
void bl_main() __attribute__ ((section (".bl_text")));
void bl_rs485_send_block() __attribute__ ((section (".bl_text")));
void bl_rs485_send_packet(uint8_t pkt_type, uint16_t payload_len) 
       __attribute__ ((section (".bl_text")));
void bl_rs485_handle_byte(uint8_t b) __attribute__ ((section (".bl_text")));
void bl_rs485_process_packet() __attribute__ ((section (".bl_text")));
void bl_disable_irq() __attribute__ ((section(".bl_text")));
void bl_nvic_disable_irq(IRQn_Type IRQn) __attribute__ ((section(".bl_text")));
int __attribute__((section (".bl_id"))) g_bl_rs485_address = RS485_ADDRESS;
uint8_t  __attribute__((section (".bl_bss"))) __attribute__((aligned(16))) bl_pkt_data[MAX_BL_RX_LEN];
uint8_t  __attribute__((section (".bl_bss"))) __attribute__((aligned(16))) bl_tx_pkt_buf[MAX_BL_TX_LEN];

enum bl_parser_state_t { BL_ST_IDLE = 0, BL_ST_HEADER = 1,
                                BL_ST_ADDRESS = 2, BL_ST_LEN_1 = 3,
                                BL_ST_LEN_2 = 4, BL_ST_TYPE = 5,
                                BL_ST_DATA = 6, BL_ST_CRC_1 = 7,
                                BL_ST_CRC_2 = 8 };
static enum bl_parser_state_t __attribute__((section(".bl_bss"))) bl_parser_state;
static uint32_t __attribute__((section(".bl_bss"))) bl_pkt_start_time;
static uint8_t  __attribute__((section(".bl_bss"))) bl_pkt_streaming;
static uint8_t  __attribute__((section(".bl_bss"))) bl_pkt_addr;
static uint16_t __attribute__((section(".bl_bss"))) bl_pkt_len;
static uint8_t  __attribute__((section(".bl_bss"))) bl_pkt_type;
static uint16_t __attribute__((section(".bl_bss"))) bl_pkt_write_idx;
static uint16_t __attribute__((section(".bl_bss"))) bl_pkt_crc;
static int      __attribute__((section(".bl_bss"))) bl_boot_requested;
static int      __attribute__((section(".bl_bss"))) bl_autoboot_countdown;
static int      __attribute__((section(".bl_bss"))) bl_autoboot_enabled;

void bl_main() 
{
  volatile int i, j;
  volatile uint32_t *p_bl_zero;
  volatile uint32_t *p;
  EFC->EEFC_FMR = EEFC_FMR_FWS(3); // set flash wait states so it can handle 
                                   // our blazing speed. otherwise we fail...
  WDT->WDT_MR = WDT_MR_WDDIS; // buh bye watchdog
  // wipe out our BSS
  for (p_bl_zero = &_bl_szero; p_bl_zero < &_bl_ezero;)
    *p_bl_zero++ = 0;
  // set up VTOR to point to our vector table
  p = (uint32_t *)&_bl_sfixed;
  //SCB->VTOR = ( (uint32_t)p & SCB_VTOR_TBLOFF_Msk ) ;
  //if ( ((uint32_t)p >= IRAM_ADDR) && ((uint32_t)pSrc < IRAM_ADDR+IRAM_SIZE) )
  //  {
	//    SCB->VTOR |= 1 << SCB_VTOR_TBLBASE_Pos ;
  //  }

  PMC->PMC_PCER0 = (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_USART0);
  bl_nvic_disable_irq(USART0_IRQn);
  bl_nvic_disable_irq(PIOA_IRQn);
  bl_nvic_disable_irq(PIOB_IRQn);
  bl_disable_irq();
  PIOA->PIO_IDR  = 0xffffffff;
  PIOB->PIO_IDR  = 0xffffffff;
  PIOA->PIO_CODR = 1 << PIN_A_LED;
  //PIOB->PIO_CODR = 1 << PIN_A_LED;
  PIOA->PIO_OER  = 1 << PIN_A_LED;
  PIOA->PIO_PER  = 1 << PIN_A_LED;
  PIOA->PIO_CODR = 1 << PIN_A_PHALANGE_POWER;
  PIOA->PIO_OER  = 1 << PIN_A_PHALANGE_POWER;
  PIOA->PIO_PER  = 1 << PIN_A_PHALANGE_POWER;

  // switch to the slow internal RC oscillator so we can monkey
  // around with the main crystal oscillator and PLL
  PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
                  PMC_MCKR_CSS_MAIN_CLK;
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |     // "password" hard-wired in logic
                  CKGR_MOR_MOSCXTST(0x10) | // startup time: slowclock*8*this
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN; // crystal oscillator enable (not select)
  //while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) { } // spin...

  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) { } // spin until stable
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  for (i = 0; i < 50000; i++) { } // idiocy

  for (j = 0; j < 2; j++)
  {
    for (i = 0; i < 30000; i++)
      PIOA->PIO_SODR = 1 << PIN_A_LED;
    for (i = 0; i < 30000; i++)
      PIOA->PIO_CODR = 1 << PIN_A_LED;
  }

  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |     // "password" hard-wired in logic
                  CKGR_MOR_MOSCXTST(0x10) | // startup time: slowclock*8*this
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN; // main crystal oscillator enable
  while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) { } // spin...

  // switch to main crystal oscillator
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |
                  CKGR_MOR_MOSCXTST(0x10) |
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN |
                  CKGR_MOR_MOSCSEL;
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS) ||
         !(PMC->PMC_SR & PMC_SR_MCKRDY)       ) { } // spin until stable

  // blink = win
  for (j = 0; j < 20; j++)
  {
    for (i = 0; i < 30000; i++)
      PIOA->PIO_SODR = 1 << PIN_A_LED;
    for (i = 0; i < 30000; i++)
      PIOA->PIO_CODR = 1 << PIN_A_LED;
  }

  PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
                   PMC_MCKR_CSS_MAIN_CLK; // select main clock (really needed?)
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  // now, spin up the PLL
  PMC->CKGR_PLLAR = CKGR_PLLAR_STUCKTO1 | // as per datasheet, must set 1<<29
                    CKGR_PLLAR_MULA(0x07) | // was 0x03
                    CKGR_PLLAR_DIVA(0x01) |
                    CKGR_PLLAR_PLLACOUNT(0x01);
  while (!(PMC->PMC_SR & PMC_SR_LOCKA)) { } // spin until lock
  // is this step needed? Atmel's EK does it...
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_MAIN_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  // finally, switch to PLL output
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  for (i = 0; i < 200000; i++) { } // idiocy
  for (j = 0; j < 3; j++)
  {
    for (i = 0; i < 100000; i++)
      PIOA->PIO_SODR = 1 << PIN_A_LED;
    for (i = 0; i < 100000; i++)
      PIOA->PIO_CODR = 1 << PIN_A_LED;
  }


  //////////////////////////////////////////////////////////////////////
  // rs485 init
  for (i = 0; i < 1000; i++) { } // idiocy
  PIOA->PIO_IDR  = 0xffffffff;
  PIOB->PIO_IDR  = 0xffffffff;
  PIOA->PIO_CODR =  (1 << PIN_A_RS485_RE) | (1 << PIN_A_RS485_DE);
  PIOA->PIO_OER  =  (1 << PIN_A_RS485_RE) | (1 << PIN_A_RS485_DE) | 
                    (1 << PIN_A_RS485_DI);
  PIOA->PIO_PER  =  (1 << PIN_A_RS485_RE) | (1 << PIN_A_RS485_DE);
  PIOA->PIO_PDR =   (1 << PIN_A_RS485_RO) | (1 << PIN_A_RS485_DI); 
  PIOA->PIO_ABCDSR[0] &= ~((1 << PIN_A_RS485_RO) | (1 << PIN_A_RS485_DI));
  PIOA->PIO_ABCDSR[1] &= ~((1 << PIN_A_RS485_RO) | (1 << PIN_A_RS485_DI));
  USART0->US_CR = US_CR_RSTRX | US_CR_RSTTX | 
                  US_CR_RXDIS | US_CR_TXDIS; // reset uart
  USART0->US_MR = US_MR_CHRL_8_BIT | US_MR_PAR_NO; // 8N1, normal mode
  USART0->US_BRGR = F_CPU / 1000000 / 16; // set baud rate
  USART0->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS; // disable DMA
  USART0->US_CR = US_CR_TXEN | US_CR_RXEN; // eanble TX and RX
  USART0->US_IDR = 0xffffffff;
  //UART1->UART_IER = UART_IER_RXRDY; // enable RX interrupt

  bl_parser_state = BL_ST_IDLE;
  bl_pkt_start_time = 0;
  bl_pkt_streaming = 0;
  bl_pkt_addr = 0;
  bl_pkt_len = 0;
  bl_pkt_type = 0;
  bl_pkt_write_idx = 0;
  bl_pkt_crc = 0;
  bl_boot_requested = 0;
  bl_autoboot_enabled = 1; 
  bl_autoboot_countdown = BL_AUTOBOOT_COUNT;
  EFC->EEFC_FMR &= ~((uint32_t)EEFC_FMR_FRDY); // no interrupt on flash ready
  while (!bl_boot_requested)
  {
    // poll for new and exciting characters on rs485
    if (USART0->US_CSR & US_CSR_RXRDY)
      bl_rs485_handle_byte(USART0->US_RHR);
    --bl_autoboot_countdown;
    if (bl_autoboot_enabled && bl_autoboot_countdown == 0)
      bl_boot_requested = 1;

    if (bl_autoboot_countdown % 30000 == 0)
    {
      if (PIOA->PIO_ODSR & (1 << PIN_A_LED))
        PIOA->PIO_CODR = (1 << PIN_A_LED);
      else 
        PIOA->PIO_SODR = (1 << PIN_A_LED);
    }
/*
    if (bl_autoboot_countdown % 1000 == 0)
    {
      volatile uint8_t dummy = 0x55;
      bl_rs485_send_block(&dummy, 1);
    }
*/
  }
 
/*
  for (j = 0; j < 20; j++)
  {
    for (i = 0; i < 100000; i++)
    {
      PIOA->PIO_SODR = 1 << PIN_A_LED;
    }
    for (i = 0; i < 100000; i++)
    {
      PIOA->PIO_CODR = 1 << PIN_A_LED;
    }
    if (j == g_bl_rs485_address)
    {
      uint8_t msg = 'H';
      bl_rs485_send_block(&msg, 1);
    }
  }
  */
}

void bl_rs485_send_block(uint8_t *block, uint32_t len)
{
  volatile uint32_t d = 0;
  // commandeer the rs485 bus
  USART0->US_CR |= US_CR_RXDIS;
  PIOA->PIO_SODR = (1 << PIN_A_RS485_RE) | (1 << PIN_A_RS485_DE);
  for (d = 0; d < 500; d++) { }
  while ((USART0->US_CSR & US_CSR_TXEMPTY) == 0) { }
  for (volatile uint32_t i = 0; i < len; i++)
  {
    USART0->US_THR = block[i];
    while ((USART0->US_CSR & US_CSR_TXEMPTY) == 0) { }
    for (volatile uint32_t j = 0; j < 100; j++) { }
  }
  for (d = 0; d < 500; d++) { }
  // release the rs485 bus
  PIOA->PIO_CODR = (1 << PIN_A_RS485_RE) | (1 << PIN_A_RS485_DE);
  USART0->US_CR &= ~US_CR_RXDIS;
  USART0->US_CR |= US_CR_RXEN;
}

void bl_rs485_handle_byte(uint8_t b)
{
  // TODO: check if we've been waiting for this packet for a while, and bail
  switch(bl_parser_state)
  {
    case BL_ST_IDLE:
      if (b == 0x42)
      {
        bl_parser_state = BL_ST_HEADER;
        bl_pkt_streaming = 1;
        bl_pkt_start_time = 0; // TODO: grab system time
      }
      break;
    case BL_ST_HEADER:
      bl_pkt_addr = b;
      bl_parser_state = BL_ST_LEN_1;
      break;
    case BL_ST_LEN_1:
      bl_pkt_len = b;
      bl_parser_state = BL_ST_LEN_2;
      break;
    case BL_ST_LEN_2:
      bl_pkt_len |= ((uint16_t)b << 8);
      bl_parser_state = BL_ST_TYPE;
      break;
    case BL_ST_TYPE:
      bl_pkt_type = b;
      bl_pkt_write_idx = 0;
      if (bl_pkt_len > 0)
        bl_parser_state = BL_ST_DATA;
      else
        bl_parser_state = BL_ST_CRC_1;
      break;
    case BL_ST_DATA:
      if (bl_pkt_write_idx < MAX_BL_RX_LEN)
        bl_pkt_data[bl_pkt_write_idx++] = b;
      if (bl_pkt_write_idx >= bl_pkt_len)
        bl_parser_state = BL_ST_CRC_1;
      break;
    case BL_ST_CRC_1:
      bl_pkt_crc = b;
      bl_parser_state = BL_ST_CRC_2;
      break;
    case BL_ST_CRC_2:
      bl_pkt_crc |= ((uint16_t)b << 8);
      bl_rs485_process_packet();
      bl_parser_state = BL_ST_IDLE;
      break;
    default:
      bl_parser_state = BL_ST_IDLE;
      break;
  }
}

void bl_rs485_process_packet() // spaghetti code just to have fewer indents...
{
  uint16_t crc = 0;
  uint8_t d, crc_highbit;

  if (bl_pkt_addr != g_bl_rs485_address && bl_pkt_addr != 0xff) 
    return;

  for (uint32_t i = 0; i < (uint32_t)bl_pkt_len+5; i++)
  {
    if (i == 0)
      d = 0x42;
    else if (i == 1)
      d = bl_pkt_addr;
    else if (i == 2)
      d = bl_pkt_len & 0xff;
    else if (i == 3)
      d = (bl_pkt_len >> 8) & 0xff;
    else if (i == 4)
      d = bl_pkt_type;
    else
      d = bl_pkt_data[i-5];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc_highbit = (crc >> 8) & 0x80;
      crc <<= 1;
      if ((d & 0x80) ^ crc_highbit)
        crc ^= 0x1021; // CRC-16 CCITT polynomial
      d <<= 1;
    }
  }
  if (bl_pkt_crc != crc)
  {
    //printf("packet with type %d address %d rejected due to checksum error\r\n",
    //       pkt_type, pkt_addr);
    return;
  }
  //printf("packet type = %02x\r\n", pkt_type);

  if (bl_pkt_type == 0x01)      // ping packet. respond with ping back.
    bl_rs485_send_packet(0x01, 0);
  else if (bl_pkt_type == 0x08) // boot
  {
    bl_rs485_send_packet(8, 0);
    bl_boot_requested = 1;
  }
  else if (bl_pkt_type == 0x0a) // read some on-chip flash
  {
    volatile int a;
    uint32_t page_num =  bl_pkt_data[0]        | (bl_pkt_data[1] <<  8) |
                        (bl_pkt_data[2] << 16) | (bl_pkt_data[3] << 24);
    if (page_num > 1024)
    { 
      bl_rs485_send_packet(0x0a, 0); // bogus request: outside flash 
      return;
    }
    // request is sane, so sniff around in flash and send it back
    bl_autoboot_enabled = 0;
    for (a = 0; a < 256; a++)
      *((uint32_t *)(bl_tx_pkt_buf + 5 + 4*a)) = 
                *((uint32_t *)(0x400000 + page_num*256 + 4*a));
    bl_rs485_send_packet(0x0a, 256);
  }
  else if (bl_pkt_type == 0x0b) // write on-chip flash page
  {
    volatile uint32_t page_num =  bl_pkt_data[0]        | 
                                 (bl_pkt_data[1] <<  8) |
                                 (bl_pkt_data[2] << 16) | 
                                 (bl_pkt_data[3] << 24) ;
    volatile uint32_t *write_addr, *read_addr;
    volatile uint32_t write_count;
    *((uint32_t *)(bl_tx_pkt_buf + 5)) = page_num; // tx page number back
    if (page_num < 16 || page_num >= 1024)
    {
      *((uint32_t *)(bl_tx_pkt_buf + 5)) = page_num;
      bl_tx_pkt_buf[9] = 0x01; // error flag
      bl_rs485_send_packet(0x0b, 5); // send nack
      return; // buh bye
    }
    //page_num = 16;
    //bl__disable_irq();
    bl_autoboot_enabled = 0;
    // stuff the write buffer
    //for (write_ofs = 0; write_ofs < 256; write_ofs += 4)
    {
      //volatile uint32_t write_addr = 0x400000 + 256 * page_num + write_ofs;
      //*((uint32_t *)write_addr) = 0x12345678; //*((uint32_t *)(bl_pkt_data + write_ofs + 4));
    }
    // issue the write command
    //write_ofs = EFC->EEFC_FSR; // bogus read of EEFC_FSR to clear it

    EFC->EEFC_FMR = EEFC_FMR_FWS(6); // as per errata in datasheet
    read_addr = (uint32_t *)(bl_pkt_data + 4);
    write_addr = (uint32_t *)(0x400000 + 256 * page_num);
    for (write_count = 0; write_count < 64; write_count++)
      *write_addr++ = *read_addr++;
    while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // spin...
    {
      /* Pointer on IAP function in ROM */
      #define CHIP_FLASH_IAP_ADDRESS (0x00800008)
      const static uint32_t (*IAP_PerformCommand)( uint32_t, uint32_t ) ;
      IAP_PerformCommand = (uint32_t (*)( uint32_t, uint32_t )) *((uint32_t*)CHIP_FLASH_IAP_ADDRESS ) ;
      IAP_PerformCommand(0,
                         EEFC_FCR_FKEY(0x5A) | 
                         EEFC_FCR_FARG(page_num) | 
                         EEFC_FCR_FCMD(EFC_FCMD_EWP));
    }
    while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // spin...
    EFC->EEFC_FMR = EEFC_FMR_FWS(2); // reset it so we can run faster now
    bl_tx_pkt_buf[9] = 0x00; // success flag
    bl_rs485_send_packet(0x0b, 5); // send ack
  }
  else if (bl_pkt_type == 0x0c) // cancel auto-boot
  {
    bl_autoboot_enabled = 0;
    bl_rs485_send_packet(0x0c, 0); // send ack
  }
}

void bl_rs485_send_packet(uint8_t pkt_type, uint16_t payload_len) 
{
  uint16_t crc = 0, i, d, crc_highbit;
  bl_tx_pkt_buf[0] = 0x42;
  bl_tx_pkt_buf[1] = 0x00; // address to master
  bl_tx_pkt_buf[2] = payload_len & 0xff;
  bl_tx_pkt_buf[3] = (payload_len >> 8) & 0xff;
  bl_tx_pkt_buf[4] = pkt_type; 
  for (i = 0; i < payload_len+5; i++)
  {
    d = bl_tx_pkt_buf[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc_highbit = (crc >> 8) & 0x80;
      crc <<= 1;
      if ((d & 0x80) ^ crc_highbit)
        crc ^= 0x1021; // CRC-16 CCITT polynomial
      d <<= 1;
    }
  }
  bl_tx_pkt_buf[5+payload_len] = crc & 0xff;
  bl_tx_pkt_buf[6+payload_len] = (crc >> 8) & 0xff;
  bl_rs485_send_block(bl_tx_pkt_buf, 7+payload_len);
}

void bl_disable_irq() 
{
  __ASM volatile("cpsid i");
}

void bl_nvic_disable_irq(IRQn_Type IRQn) 
{
  NVIC->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

