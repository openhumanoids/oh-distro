/*  Software License Agreement (Apache License)
 *
 *  Copyright 2012 Open Source Robotics Foundation
 *  Author: Morgan Quigley
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

//char bl_flash_page[512] __attribute__ ((section("bootloader_bss"))) = {0};
//char bl_stack[1000] __attribute__ ((section("bootloader_stack"))) = {0};
#include "sam3x.h"
#include <stdbool.h>

#define MAX_BL_RX_LEN 1024
#define MAX_BL_TX_LEN 1024
#define BL_AUTOBOOT_COUNT 100000

#define BL_LED_PIO PIOA
#define BL_LED_PIN 23
#define BL_RS485_DE_PIO PIOB
#define BL_RS485_DE_PIN 25
#define PIN_A_RS485_RO 12
#define PIN_A_RS485_DI 13

// make up something for the motherboard...
#define RS485_ADDRESS 20

#if (!defined(BL_LED_PIO) || !defined(BL_LED_PIN))
  #error bootloader LED port or pin not defined
#endif

#if (!defined(BL_RS485_DE_PIO) || !defined(BL_RS485_DE_PIN))
  #error bootloader rs485 driver-enable port not defined
#endif

extern uint32_t _bl_sfixed;
extern uint32_t _bl_szero;
extern uint32_t _bl_ezero;
void bl_main() __attribute__ ((section (".bl_text")));
//void bl_rs485_send_block() __attribute__ ((section (".bl_text")));
void bl_rs485_send_packet(uint8_t pkt_type, uint16_t payload_len) 
       __attribute__ ((section (".bl_text")));
void bl_rs485_handle_byte(uint8_t b) __attribute__ ((section (".bl_text")));
void bl_rs485_process_packet() __attribute__ ((section (".bl_text")));
//void bl_disable_irq() __attribute__ ((section(".bl_text")));
//void bl_nvic_disable_irq(IRQn_Type IRQn) __attribute__ ((section(".bl_text")));
bool bl_write_flash_page(uint16_t page_num, uint8_t *data, uint8_t *write_error) __attribute__ ((section(".bl_text")));
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
//static uint8_t  __attribute__((section(".bl_bss"))) bl_pkt_streaming;
static uint8_t  __attribute__((section(".bl_bss"))) bl_pkt_addr;
static uint16_t __attribute__((section(".bl_bss"))) bl_pkt_len;
static uint8_t  __attribute__((section(".bl_bss"))) bl_pkt_type;
static uint16_t __attribute__((section(".bl_bss"))) bl_pkt_write_idx;
static uint16_t __attribute__((section(".bl_bss"))) bl_pkt_crc;
static int      __attribute__((section(".bl_bss"))) bl_boot_requested;
static int      __attribute__((section(".bl_bss"))) bl_autoboot_countdown;
static int      __attribute__((section(".bl_bss"))) bl_autoboot_enabled;
static uint8_t  __attribute__((section(".bl_bss"))) bl_flash_page_buf[256];
static uint8_t  __attribute__((section(".bl_bss"))) bl_flash_word_ready[64];

void bl_main() 
{
  return;
  volatile int i, j;
  //volatile uint32_t *p;
  EFC0->EEFC_FMR = EEFC_FMR_FWS(4); // set flash wait states so it can handle 
  EFC1->EEFC_FMR = EEFC_FMR_FWS(4); // our blazing speed. otherwise we fail...
  WDT->WDT_MR = WDT_MR_WDDIS; // buh bye watchdog
  //for (volatile uint32_t i = 0; i < 10000000; i++) { }
  // wipe out our BSS
  for (volatile uint32_t *p_bl_zero = &_bl_szero; p_bl_zero < &_bl_ezero;)
    *p_bl_zero++ = 0;
  // set up VTOR to point to our vector table. but wait doesn't reset do this?
  //p = (uint32_t *)&_bl_sfixed;
  //SCB->VTOR = ( (uint32_t)p & SCB_VTOR_TBLOFF_Msk ) ;
  //if ( ((uint32_t)p >= IRAM_ADDR) && ((uint32_t)pSrc < IRAM_ADDR+IRAM_SIZE) )
  //  {
	//    SCB->VTOR |= 1 << SCB_VTOR_TBLBASE_Pos ;
  //  }

  __ASM volatile("cpsid i"); // disable all interrupts
  PMC->PMC_PCER0 = (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_PIOC) | 
                   (1 << ID_USART1);
  BL_LED_PIO->PIO_CODR = BL_LED_PIO->PIO_OER = BL_LED_PIO->PIO_PER = 
                                                             1 << BL_LED_PIN;
  BL_LED_PIO->PIO_SODR = 1 << BL_LED_PIN;
  //for (volatile uint32_t i = 0; i < 10000000; i++) { }
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
  for (i = 0; i < 50000; i++) { } // can't remember why i did this. maybe
                                  // to waste some time in case JTAG needs
                                  // to connect in case the following code
                                  // is borked somehow.
  for (j = 0; j < 2; j++) // blink the LED a few times just to say we're alive
  {
    for (i = 0; i < 30000; i++) { }
    BL_LED_PIO->PIO_SODR = 1 << BL_LED_PIN;
    for (i = 0; i < 30000; i++) { }
    BL_LED_PIO->PIO_CODR = 1 << BL_LED_PIN;
  }
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |     // "password" hard-wired in logic
                  CKGR_MOR_MOSCXTST(0x10) | // startup time: slowclock*8*this
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN; // main crystal oscillator enable
  while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) { } // spin...

  // switch to main crystal oscillator
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37)      |
                  CKGR_MOR_MOSCXTST(0x10) |
                  CKGR_MOR_MOSCRCEN       | // keep on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN       |
                  CKGR_MOR_MOSCSEL;
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS) ||
         !(PMC->PMC_SR & PMC_SR_MCKRDY)       ) { } // spin until stable
  // blink somewhat faster to show crystal oscillator is running
  for (j = 0; j < 4; j++)
  {
    for (i = 0; i < 30000; i++) { }
    BL_LED_PIO->PIO_SODR = 1 << BL_LED_PIN;
    for (i = 0; i < 30000; i++) { }
    BL_LED_PIO->PIO_CODR = 1 << BL_LED_PIN;
  }
  PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
                   PMC_MCKR_CSS_MAIN_CLK; // select main clock (really needed?)
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  // now, spin up the PLL. we want PLL output to be 128 MHz
  PMC->CKGR_PLLAR = CKGR_PLLAR_ONE        | // as per datasheet, must set 1<<29
                    CKGR_PLLAR_MULA(0x07) | // pll = crystal * (mul+1)/div
                    CKGR_PLLAR_DIVA(0x01) | // which gives us 128 mhz
                    CKGR_PLLAR_PLLACOUNT(0x01);
  while (!(PMC->PMC_SR & PMC_SR_LOCKA)) { } // spin until lock
  // is this step needed? Atmel's EK does it...
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_MAIN_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  // finally, switch to PLL output, dividing by 2 to get us 64 MHz PLLACK
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  for (i = 0; i < 200000; i++) { } // why is this here?
  for (j = 0; j < 3; j++) // blink a few times to show we're on 64 MHz clock
  {
    for (i = 0; i < 100000; i++) { }
    BL_LED_PIO->PIO_SODR = 1 << BL_LED_PIN;
    for (i = 0; i < 100000; i++) { }
    BL_LED_PIO->PIO_CODR = 1 << BL_LED_PIN;
  }

  //////////////////////////////////////////////////////////////////////
  // rs485 init
  //for (i = 0; i < 1000; i++) { } // idiocy
  BL_RS485_DE_PIO->PIO_CODR = 
      BL_RS485_DE_PIO->PIO_OER  = 
      BL_RS485_DE_PIO->PIO_PER  = (1 << BL_RS485_DE_PIN);
  PIOA->PIO_OER  =  (1 << PIN_A_RS485_DI); // output enable
  PIOA->PIO_PDR  =  (1 << PIN_A_RS485_RO) | (1 << PIN_A_RS485_DI); // periph
  PIOA->PIO_ABSR &= ~((1 << PIN_A_RS485_RO) | (1 << PIN_A_RS485_DI)); // A
  USART1->US_CR = US_CR_RSTRX | US_CR_RSTTX | 
                  US_CR_RXDIS | US_CR_TXDIS; // reset uart
  USART1->US_MR = US_MR_CHRL_8_BIT | US_MR_PAR_NO; // 8N1, normal mode
  USART1->US_BRGR = F_CPU / 1000000 / 16;
  USART1->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS; // disable DMA
  //USART1->US_IDR = 0xffffffff;
  USART1->US_CR = US_CR_TXEN | US_CR_RXEN; // eanble TX and RX
  /*
  // these guys are already initialized to zero because they're in bss
  bl_parser_state = BL_ST_IDLE;
  bl_pkt_start_time = 0;
  bl_pkt_streaming = 0;
  bl_pkt_addr = 0;
  bl_pkt_len = 0;
  bl_pkt_type = 0;
  bl_pkt_write_idx = 0;
  bl_pkt_crc = 0;
  bl_boot_requested = 0;
  */
  bl_autoboot_enabled = 1;
  bl_autoboot_countdown = BL_AUTOBOOT_COUNT;
  EFC0->EEFC_FMR &= ~((uint32_t)EEFC_FMR_FRDY); // no interrupt on flash ready
  EFC1->EEFC_FMR &= ~((uint32_t)EEFC_FMR_FRDY); // no interrupt on flash ready
  //bl_reset_flash_page_buf(); // this is initialized to zero since it's in bss
  while (!bl_boot_requested)
  {
    // poll for new and exciting characters on rs485
    if (USART1->US_CSR & US_CSR_RXRDY)
    {
      //bl_rs485_handle_byte(USART1->US_RHR);
      const uint8_t b = USART1->US_RHR;
      switch(bl_parser_state)
      {
        case BL_ST_IDLE:
          if (b == 0x42)
          {
            bl_parser_state = BL_ST_HEADER;
            //bl_pkt_streaming = 1;
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
    --bl_autoboot_countdown;
    if (bl_autoboot_enabled && bl_autoboot_countdown == 0)
      bl_boot_requested = 1;

    if (bl_autoboot_countdown % 20000 == 0)
    {
      if (BL_LED_PIO->PIO_ODSR & (1 << BL_LED_PIN))
        BL_LED_PIO->PIO_CODR = (1 << BL_LED_PIN);
      else 
        BL_LED_PIO->PIO_SODR = (1 << BL_LED_PIN);
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

#if 0
void bl_rs485_send_block(uint8_t *block, uint32_t len)
{
  //volatile uint32_t d = 0;
  // commandeer the rs485 bus
}
#endif

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

  // page_num in first 4 bytes is used in several places. just do it once here.
  const uint32_t page_num =  bl_pkt_data[0]        | (bl_pkt_data[1] <<  8) |
                            (bl_pkt_data[2] << 16) | (bl_pkt_data[3] << 24);

  if (bl_pkt_type == 0x00)      // ping packet. respond with ping back.
    bl_rs485_send_packet(0, 0);
  else if (bl_pkt_type == 0x08) // boot
  {
    bl_rs485_send_packet(8, 0);
    bl_boot_requested = 1;
  }
  else if (bl_pkt_type == 0x0a) // read some on-chip flash
  {
    volatile int a;
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
    *((uint32_t *)(bl_tx_pkt_buf + 5)) = page_num; // tx page number back
    // mark all words as being ready
    for (int i = 0; i < 64; i++)
      bl_flash_word_ready[i] = true;
    uint8_t write_error = 1;
    if (bl_write_flash_page(page_num, bl_pkt_data+4, &write_error))
    {
      bl_autoboot_enabled = 0;
      bl_tx_pkt_buf[9] = 0x00; // success flag
      bl_rs485_send_packet(0x0b, 5); // send ack
    }
    else
    {
      bl_tx_pkt_buf[9] = write_error; // error flag
      bl_rs485_send_packet(0x0b, 5); // send nack
    }
  }
  else if (bl_pkt_type == 0x0c) // cancel auto-boot
  {
    bl_autoboot_enabled = 0;
    bl_rs485_send_packet(0x0c, 0); // send ack
  }
  else if (bl_pkt_type == 0x0d) // set flash word
  {
    uint8_t word_num  = bl_pkt_data[0];
    if (word_num < 64)
    {
      bl_autoboot_enabled = 0;
      bl_flash_word_ready[word_num] = true;
      for (int i = 0; i < 4; i++)
        bl_flash_page_buf[word_num * 4 + i] = bl_pkt_data[1+i];
      bl_tx_pkt_buf[6] = 0x00; // success
    }
    else
      bl_tx_pkt_buf[6] = 0x01; // error flag
    bl_tx_pkt_buf[5] = word_num; // tx word number back
    bl_rs485_send_packet(0x0d, 2); // send response
  }
  else if (bl_pkt_type == 0x0e) // write page buffer
  {
    *((uint32_t *)(bl_tx_pkt_buf + 5)) = page_num; // tx page number back
    uint8_t write_error = 0;
    if (bl_write_flash_page(page_num, bl_flash_page_buf, &write_error)) // checks in func.
      bl_tx_pkt_buf[9] = 0x00; // success flag
    else
      bl_tx_pkt_buf[9] = write_error; //0x01; // error flag
    bl_rs485_send_packet(0x0e, 5); // send ack
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
  //bl_rs485_send_block(bl_tx_pkt_buf, 7+payload_len);
  USART1->US_CR |= US_CR_RXDIS;
  BL_RS485_DE_PIO->PIO_SODR = (1 << BL_RS485_DE_PIN);
  //for (d = 0; d < 500; d++) { }
  while ((USART1->US_CSR & US_CSR_TXEMPTY) == 0) { }
  for (volatile uint32_t i = 0; i < (7+payload_len); i++)
  {
    USART1->US_THR = bl_tx_pkt_buf[i];
    while ((USART1->US_CSR & US_CSR_TXEMPTY) == 0) { }
  }
  //for (d = 0; d < 500; d++) { }
  // release the rs485 bus
  BL_RS485_DE_PIO->PIO_CODR = (1 << BL_RS485_DE_PIN);
  USART1->US_CR &= ~US_CR_RXDIS;
  USART1->US_CR |= US_CR_RXEN;
}

/*
void bl_nvic_disable_irq(IRQn_Type IRQn) 
{
  NVIC->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
*/
bool bl_write_flash_page(const uint16_t page_num, uint8_t *data, uint8_t *err)
{
  volatile uint32_t write_count;
  volatile uint32_t *write_addr, *read_addr;
  if (page_num < 16 || page_num >= 1024)
  {
    *err = 1;
    return false;
  }
  // check to see if we've got all words ready in the buffer
  for (int i = 0; i < 64; i++)
    if (!bl_flash_word_ready[i])
    {
      *err = i + 0x80;
      return false;
    }
  EFC0->EEFC_FMR = EEFC_FMR_FWS(6); // as per errata in datasheet
  EFC1->EEFC_FMR = EEFC_FMR_FWS(6); // as per errata in datasheet
  volatile uint32_t bank_id = (page_num < 1024 ? 0 : 1);
  volatile Efc *efc = (bank_id == 0 ? EFC0 : EFC1);
  read_addr = (uint32_t *)data; 
  write_addr = (uint32_t *)(0x400000 + 256 * page_num);
  for (write_count = 0; write_count < 64; write_count++)
    *write_addr++ = *read_addr++;
  while ((efc->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // spin...
  // gratuitous hack to jump to the IAP function in ROM to write a flash page
  typedef uint32_t (*iap_fp)(uint32_t, uint32_t);
  iap_fp iap = *((iap_fp *)(IROM_ADDR + 8)); // magic IAP pointer in ROM.
  iap(bank_id, EEFC_FCR_FKEY(0x5A) | EEFC_FCR_FARG(page_num) | 
               EEFC_FCR_FCMD(0x03));
  while ((efc->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // spin...
  EFC0->EEFC_FMR = EEFC_FMR_FWS(3); // reset it so we can run faster now
  EFC1->EEFC_FMR = EEFC_FMR_FWS(3); // reset it so we can run faster now
  for (int i = 0; i < 64; i++)
    bl_flash_word_ready[i] = 0;
  for (int i = 0; i < 256; i++)
    bl_flash_page_buf[i] = 0;

  return true;
}

