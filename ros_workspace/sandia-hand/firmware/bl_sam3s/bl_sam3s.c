#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include <stdbool.h>

#define MAX_BL_RX_LEN 1024
#define MAX_BL_TX_LEN 1024
#define BL_AUTOBOOT_COUNT 3000000

#include "bl_stubs.h"

extern uint32_t _szero, _ezero;
void send_packet(uint8_t pkt_type, uint16_t payload_len);
void process_packet();
bool write_flash_page(uint16_t page_num, uint8_t *data, uint8_t *write_error);
const int __attribute__((section (".bl_crush_start"))) g_bl_crush_start = 0;
uint8_t  __attribute__((aligned(16))) g_pkt_data[MAX_BL_RX_LEN];
uint8_t  __attribute__((aligned(16))) g_tx_pkt_buf[MAX_BL_TX_LEN];

enum { BL_ST_IDLE = 0,  BL_ST_HEADER = 1, BL_ST_ADDRESS = 2, BL_ST_LEN_1 = 3,
       BL_ST_LEN_2 = 4, BL_ST_TYPE = 5,   BL_ST_DATA = 6,    BL_ST_CRC_1 = 7,
       BL_ST_CRC_2 = 8 } g_parser_state;
static uint32_t g_pkt_start_time, g_time;
static uint8_t  g_pkt_streaming, g_pkt_addr, g_pkt_type;
static uint16_t g_pkt_len, g_pkt_write_idx, g_pkt_crc;
static int      g_boot_requested, g_autoboot_countdown, g_autoboot_enabled;
static uint8_t  g_flash_page_buf[256], g_flash_word_ready[64];

void main() 
{
  volatile int i, j;
  EFC->EEFC_FMR = EEFC_FMR_FWS(3); // set flash wait states so it can handle 
                                   // our blazing speed. otherwise we fail...
  WDT->WDT_MR = WDT_MR_WDDIS; // buh bye watchdog
  __ASM volatile("cpsid i"); // disable all interrupts
  for (volatile uint32_t *p_zero = &_szero; p_zero < &_ezero;)
    *p_zero++ = 0; // wipe out bss
  bl_init();
  // switch to the slow internal RC oscillator so we can monkey
  // around with the main crystal oscillator and PLL
  PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
                  PMC_MCKR_CSS_MAIN_CLK;
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |     // "password" hard-wired in logic
                  CKGR_MOR_MOSCXTST(0x10) | // startup time: slowclock*8*this
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN; // crystal oscillator enable (not select)
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) { } // spin until stable
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  for (i = 0; i < 50000; i++) { } // pause a bit in case jtag needs to connect
  for (j = 0; j < 2; j++)
  {
    for (i = 0; i < 30000; i++) { }
    bl_led(BL_LED_TOGGLE);
  }
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |     // "password" hard-wired in logic
                  CKGR_MOR_MOSCXTST(0x10) | // startup time: slowclock*8*this
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN; // main crystal oscillator enable
  while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) { } // spin...
  // now, switch to main crystal oscillator
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |
                  CKGR_MOR_MOSCXTST(0x10) |
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN |
                  CKGR_MOR_MOSCSEL;
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS) ||
         !(PMC->PMC_SR & PMC_SR_MCKRDY)       ) { } // spin until stable
  for (j = 0; j < 20; j++)
  {
    for (i = 0; i < 30000; i++) { }
    bl_led(BL_LED_TOGGLE);
  }
  PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
                   PMC_MCKR_CSS_MAIN_CLK; // select main clock (really needed?)
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  // finally, spin up the PLL
  PMC->CKGR_PLLAR = CKGR_PLLAR_STUCKTO1 | // as per datasheet, must set 1<<29
                    CKGR_PLLAR_MULA(0x07) | 
                    CKGR_PLLAR_DIVA(0x01) |
                    CKGR_PLLAR_PLLACOUNT(0x01);
  while (!(PMC->PMC_SR & PMC_SR_LOCKA)) { } // spin until lock
  // is this step needed? Atmel's EK does it...
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_MAIN_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  // finally, switch to PLL output
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  for (j = 0; j < 3; j++)
  {
    for (i = 0; i < 100000; i++) { }
    bl_led(BL_LED_TOGGLE);
  }
  g_autoboot_enabled = 1;
  g_autoboot_countdown = BL_AUTOBOOT_COUNT;
  // see if there is even an application image we could load if we wanted to
  if (*(uint32_t *)(0x000402000) == 0)
    g_autoboot_enabled = 0; // nothing there. don't try to boot to it.
  EFC->EEFC_FMR &= ~((uint32_t)EEFC_FMR_FRDY); // no interrupt on flash ready
  while (!g_boot_requested)
  {
    g_time++; // todo: hook to an actual system timer
    if (g_pkt_streaming && g_time - g_pkt_start_time > 20000)
    {
      g_parser_state = BL_ST_IDLE;
      g_pkt_streaming = 0; // packet has taken too long. reset parser state.
    }
    if (bl_uart_byte_ready()) // poll for new and exciting characters 
    {
      const uint8_t b = bl_uart_get_byte();
      switch(g_parser_state)
      {
        case BL_ST_IDLE:
          if (b == 0x42)
          {
            g_parser_state = BL_ST_HEADER;
            g_pkt_streaming = 1;
            g_pkt_start_time = g_time;
          }
          break;
        case BL_ST_HEADER:
          g_pkt_addr = b;
          g_parser_state = BL_ST_LEN_1;
          break;
        case BL_ST_LEN_1:
          g_pkt_len = b;
          g_parser_state = BL_ST_LEN_2;
          break;
        case BL_ST_LEN_2:
          g_pkt_len |= ((uint16_t)b << 8);
          g_parser_state = BL_ST_TYPE;
          break;
        case BL_ST_TYPE:
          g_pkt_type = b;
          g_pkt_write_idx = 0;
          if (g_pkt_len > 0)
            g_parser_state = BL_ST_DATA;
          else
            g_parser_state = BL_ST_CRC_1;
          break;
        case BL_ST_DATA:
          if (g_pkt_write_idx < MAX_BL_RX_LEN)
            g_pkt_data[g_pkt_write_idx++] = b;
          if (g_pkt_write_idx >= g_pkt_len)
            g_parser_state = BL_ST_CRC_1;
          break;
        case BL_ST_CRC_1:
          g_pkt_crc = b;
          g_parser_state = BL_ST_CRC_2;
          break;
        case BL_ST_CRC_2:
          g_pkt_crc |= ((uint16_t)b << 8);
          process_packet();
          g_parser_state = BL_ST_IDLE;
          g_pkt_streaming = 0;
          break;
        default:
          g_parser_state = BL_ST_IDLE;
          g_pkt_streaming = 0;
          break;
      }
    }
    --g_autoboot_countdown;
    if (g_autoboot_enabled && g_autoboot_countdown == 0)
      g_boot_requested = 1;
    if (g_autoboot_countdown % 40000 == 0)
      bl_led(BL_LED_TOGGLE);
  }
}

void process_packet() // spaghetti code just to have fewer indents...
{
  uint16_t crc = 0;
  uint8_t d, crc_highbit;
  if (g_pkt_addr != g_rs485_addr && g_pkt_addr != 0xff) 
    return;
  // calculate the local checksum
  for (uint32_t i = 0; i < (uint32_t)g_pkt_len+5; i++)
  {
    if (i == 0)
      d = 0x42;
    else if (i == 1)
      d = g_pkt_addr;
    else if (i == 2)
      d = g_pkt_len & 0xff;
    else if (i == 3)
      d = (g_pkt_len >> 8) & 0xff;
    else if (i == 4)
      d = g_pkt_type;
    else
      d = g_pkt_data[i-5];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc_highbit = (crc >> 8) & 0x80;
      crc <<= 1;
      if ((d & 0x80) ^ crc_highbit)
        crc ^= 0x1021; // CRC-16 CCITT polynomial
      d <<= 1;
    }
  }
  if (g_pkt_crc != crc) // compare local checksum with received checksum
    return; // mismatch = corrupted packet
  // page_num in first 4 bytes is used in several places. just do it once here.
  const uint32_t page_num =  g_pkt_data[0]        | (g_pkt_data[1] <<  8) |
                            (g_pkt_data[2] << 16) | (g_pkt_data[3] << 24);
  if (g_pkt_type == 0x01)      // ping packet. respond with ping back.
    send_packet(0x01, 0);
  else if (g_pkt_type == 0x08) // boot
  {
    send_packet(8, 0);
    g_boot_requested = 1;
  }
  else if (g_pkt_type == 0x0a) // read some on-chip flash
  {
    volatile int a;
    if (page_num > 1024)
    { 
      send_packet(0x0a, 0); // bogus request: outside flash 
      return;
    }
    // request is sane, so sniff around in flash and send it back
    g_autoboot_enabled = 0;
    for (a = 0; a < 64; a++)
      *((uint32_t *)(g_tx_pkt_buf + 5 + 4*a)) = 
                          *((uint32_t *)(0x400000 + page_num*256 + 4*a));
    send_packet(0x0a, 256);
  }
  else if (g_pkt_type == 0x0b) // write on-chip flash page
  {
    *((uint32_t *)(g_tx_pkt_buf + 5)) = page_num; // tx page number back
    // mark all words as being ready
    for (int i = 0; i < 64; i++)
      g_flash_word_ready[i] = true;
    uint8_t write_error = 1;
    if (write_flash_page(page_num, g_pkt_data+4, &write_error))
    {
      g_autoboot_enabled = 0;
      g_tx_pkt_buf[9] = 0x00; // success flag
      send_packet(0x0b, 5); // send ack
    }
    else
    {
      g_tx_pkt_buf[9] = write_error; // error flag
      send_packet(0x0b, 5); // send nack
    }
  }
  else if (g_pkt_type == 0x0c) // cancel auto-boot
  {
    g_autoboot_enabled = 0;
    send_packet(0x0c, 0); // send ack
  }
  else if (g_pkt_type == 0x0d) // set flash word
  {
    uint8_t word_num  = g_pkt_data[0];
    if (word_num < 64)
    {
      g_autoboot_enabled = 0;
      g_flash_word_ready[word_num] = true;
      for (int i = 0; i < 4; i++)
        g_flash_page_buf[word_num * 4 + i] = g_pkt_data[1+i];
      g_tx_pkt_buf[6] = 0x00; // success
    }
    else
      g_tx_pkt_buf[6] = 0x01; // error flag
    g_tx_pkt_buf[5] = word_num; // tx word number back
    send_packet(0x0d, 2); // send response
  }
  else if (g_pkt_type == 0x0e) // write page buffer
  {
    *((uint32_t *)(g_tx_pkt_buf + 5)) = page_num; // tx page number back
    uint8_t write_error = 0;
    if (write_flash_page(page_num, g_flash_page_buf, &write_error)) 
      g_tx_pkt_buf[9] = 0x00; // success flag
    else
      g_tx_pkt_buf[9] = write_error; //0x01; // error flag
    send_packet(0x0e, 5); // send ack
  }
  else if (g_pkt_type == 0xfa)
  {
    // read hardware version
    *((uint32_t *)(g_tx_pkt_buf + 5)) = g_bl_hw_version; 
    send_packet(0xfa, 4); // send nack
  }
  /*
  else if (bl_pkt_type == 0xfb) // read chip serial number. todo someday...
  {
    uint32_t serial[4];
    bl_read_flash_id(serial);
    for (int i = 0; i < 16; i++)
      bl_tx_pkt_buf[5+i] = *(((uint8_t *)serial) + i);
    bl_rs485_send_packet(0xfb, 16);
  }
  */
}

void send_packet(uint8_t pkt_type, uint16_t payload_len) 
{
  uint16_t crc = 0, i, d, crc_highbit;
  g_tx_pkt_buf[0] = 0x42;
  g_tx_pkt_buf[1] = 0x00; // address to master
  g_tx_pkt_buf[2] = payload_len & 0xff;
  g_tx_pkt_buf[3] = (payload_len >> 8) & 0xff;
  g_tx_pkt_buf[4] = pkt_type; 
  for (i = 0; i < payload_len+5; i++)
  {
    d = g_tx_pkt_buf[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc_highbit = (crc >> 8) & 0x80;
      crc <<= 1;
      if ((d & 0x80) ^ crc_highbit)
        crc ^= 0x1021; // CRC-16 CCITT polynomial
      d <<= 1;
    }
  }
  g_tx_pkt_buf[5+payload_len] = crc & 0xff;
  g_tx_pkt_buf[6+payload_len] = (crc >> 8) & 0xff;
  bl_uart_tx(g_tx_pkt_buf, 7 + payload_len);
}

bool write_flash_page(const uint16_t page_num, uint8_t *data, uint8_t *err)
{
  volatile uint32_t write_count;
  volatile uint32_t *write_addr, *read_addr;
  if (page_num < 32 || page_num >= 1024)
  {
    *err = 1;
    return false;
  }
  // check to see if we've got all words ready in the buffer
  for (int i = 0; i < 64; i++)
    if (!g_flash_word_ready[i])
    {
      *err = i + 0x80;
      return false;
    }
  EFC->EEFC_FMR = EEFC_FMR_FWS(6); // as per errata in datasheet
  read_addr = (uint32_t *)data; 
  write_addr = (uint32_t *)(0x400000 + 256 * page_num);
  for (write_count = 0; write_count < 64; write_count++)
    *write_addr++ = *read_addr++;
  while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // busy wait
  typedef uint32_t (*iap_fp)(uint32_t, uint32_t);
  iap_fp iap = *((iap_fp *)0x00800008); // magic IAP pointer in ROM.
  iap(0, EEFC_FCR_FKEY(0x5A) | EEFC_FCR_FARG(page_num) | 
         EEFC_FCR_FCMD(EFC_FCMD_EWP));
  while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // busy wait
  EFC->EEFC_FMR = EEFC_FMR_FWS(2); // reset it so we can run faster now
  for (int i = 0; i < 64; i++)
    g_flash_word_ready[i] = 0;
  for (int i = 0; i < 256; i++)
    g_flash_page_buf[i] = 0;
  return true;
}
/*
void bl_read_flash_id(uint32_t *buf) // todo... finish this implementation.
{
  EFC->EEFC_FMR |= 1 << 16; // not documented. atmel lib does it though.
  EFC->EEFC_FCR = EEFC_FCR_FKEY(0x5A) | EEFC_FCR_FCMD(EFC_FCMD_STUI); // start
  while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // busy wait...
  uint32_t *flash_addr = (uint32_t *)IFLASH_ADDR; // read out the ID
  for (int i = 0; i < 4; i++)
    buf[i] = flash_addr[i]; 
  EFC->EEFC_FCR = EEFC_FCR_FKEY(0x5A) | EEFC_FCR_FCMD(EFC_FCMD_SPUI); // stop
  while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // busy wait...
  EFC->EEFC_FMR &= ~(1 << 16); // not documented. atmel lib does it though.
}
*/

