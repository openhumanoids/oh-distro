#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/stat.h>
#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "pins.h"
#include "params.h"
#include "string.h"
#include "control.h"
#include "halls.h"
#include "gpio.h"
#include "motors.h"
#include "imu.h"
#include "pb.h"
#include "comms.h"
#include "status.h"
#include "i2c_sensors.h"
#include "adc.h"

static volatile uint8_t g_tx_pkt_buf[COMMS_MAX_PACKET_LENGTH];
extern uint32_t _sid;
static int g_rs485_address = 0xfe; // bogus 
static uint32_t g_bl_hw_version = 0;    // bogus
static volatile uint8_t g_rx_buf[COMMS_RX_BUF_LEN];
static volatile unsigned g_rx_buf_writepos = 0, g_rx_buf_readpos = 0;

void pioa_irq(void) __attribute__ ((section (".ramfunc")));
void piob_irq(void) __attribute__ ((section (".ramfunc")));

/////////////////////////////////////////////////////////////////////////

void init_clocks()
{
  if (!(PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)) // if not running main oscillator
  {
    PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |     // "password" hard-wired in logic
                    CKGR_MOR_MOSCXTST(0x20) | // startup time: slowclock*8*this
                    CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                    CKGR_MOR_MOSCXTEN; // main crystal oscillator enable
    while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) { } // spin...
  }
  // switch to main crystal oscillator
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |
                  CKGR_MOR_MOSCXTST(0x20) |
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN |
                  CKGR_MOR_MOSCSEL;
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) { } // spin until stable
  PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
                  PMC_MCKR_CSS_MAIN_CLK; // select main clock (really needed?)
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  // now, spin up the PLL so we can run at 64 MHz
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
  gpio_led(false);
  g_bl_hw_version = *((uint32_t *)0x0401ff8); // magic, defined in bootloader
  if (((g_bl_hw_version >> 16) & 0xffff) != 0xbeef) // check for magic bytes
    g_bl_hw_version = 0; // older bootloaders didn't define this.
  else
    g_bl_hw_version &= 0xffff; // mask out the top magic bytes
}

void rs485_init()
{
  PMC_EnablePeripheral(ID_USART0);
  PIO_Configure(&pin_rs485_de, 1);
  PIO_Configure(&pin_rs485_di, 1);
  PIO_Configure(&pin_rs485_ro, 1);
  
  USART0->US_CR = US_CR_RSTRX | UART_CR_RSTTX | 
                  US_CR_RXDIS | UART_CR_TXDIS; // reset usart
  USART0->US_IDR = 0xffffffff; // disable all interrupts
  USART0->US_MR = US_MR_CHRL_8_BIT | US_MR_PAR_NO ; // 8N1, normal mode
  USART0->US_BRGR = F_CPU / 2000000 / 16; //1000000 / 16;
  USART0->US_CR = UART_CR_TXEN | UART_CR_RXEN; // eanble TX and RX
  USART0->US_IER = UART_IER_RXRDY; // enable RX interrupt
  NVIC_SetPriority(USART0_IRQn, 1);
  NVIC_EnableIRQ(USART0_IRQn);
  PIO_Clear(&pin_rs485_de);
  g_rs485_address = *((uint32_t *)0x0401ffc); // magic, defined in bootloader
}

void rs485_send_block(uint8_t *block, uint32_t len)
{
  volatile uint32_t d = 0;
  //volatile uint8_t dummy_block[10] = {10, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  //volatile uint32_t dummy_block_len = 10;
  //block = dummy_block;
  //len = dummy_block_len;
  USART0->US_CR |= US_CR_RXDIS;
  //for (d = 0; d < 1000; d++) { }
  PIO_Set(&pin_rs485_de);
  for (d = 0; d < 50; d++) { }
  while ((USART0->US_CSR & US_CSR_TXEMPTY) == 0) { }
  for (uint32_t i = 0; i < len; i++)
  {
    USART0->US_THR = block[i];
    while ((USART0->US_CSR & US_CSR_TXEMPTY) == 0) { }
  }
  for (d = 0; d < 50; d++) { }
  PIO_Clear(&pin_rs485_de);
  //for (d = 0; d < 1000; d++) { }
  USART0->US_CR &= ~US_CR_RXDIS;
  USART0->US_CR |= US_CR_RXEN;
  if (USART0->US_CSR & US_CSR_RXRDY)
  {
    // flush incoming buffer in case there was crap there from our TX
    USART0->US_RHR;
    //rs485_handle_byte(b); //USART0->US_RHR);
  }
}

void rs485_send_packet(uint8_t pkt_type, uint16_t payload_len)
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
  rs485_send_block((uint8_t *)g_tx_pkt_buf, 7+payload_len);
}

void rs485_process_packet(uint8_t pkt_addr, uint16_t payload_len,
                          uint8_t pkt_type, uint8_t *payload,
                          uint16_t pkt_crc)
{
  uint16_t crc = 0;
  uint8_t d, crc_highbit;

  if (pkt_addr != g_rs485_address && pkt_addr != 0xff) 
    return;

  for (uint32_t i = 0; i < (uint32_t)payload_len+5; i++)
  {
    if (i == 0)
      d = 0x42;
    else if (i == 1)
      d = pkt_addr;
    else if (i == 2)
      d = payload_len & 0xff;
    else if (i == 3)
      d = (payload_len >> 8) & 0xff;
    else if (i == 4)
      d = pkt_type;
    else
      d = payload[i-5];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc_highbit = (crc >> 8) & 0x80;
      crc <<= 1;
      if ((d & 0x80) ^ crc_highbit)
        crc ^= 0x1021; // CRC-16 CCITT polynomial
      d <<= 1;
    }
  }
  if (pkt_crc != crc)
  {
    //printf("packet with type %d address %d rejected due to checksum error\r\n",
    //       pkt_type, pkt_addr);
    return;
  }
  if (pkt_type == 0x01)     // ping packet. respond with ping back.
  {
    rs485_send_packet(0x01, 0);
  }
  else if (pkt_type == 0x09) // reset 
  {
    rs485_send_packet(9, 0); // ack the request
    RSTC->RSTC_CR = RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;
  }
  else if (pkt_type == 0x10) // inertial sensor read
  {
    for (uint8_t i = 0; i < 6; i++)
      *((uint16_t *)(&g_tx_pkt_buf[5+2*i])) = g_i2c_sensors_data[1+i];
    rs485_send_packet(0x10, 12);
  }
  else if (pkt_type == 0x12) // hall state query
  {
    //volatile uint32_t pb = REG_PIOB_PDSR;
    g_tx_pkt_buf[5] = (uint8_t)(g_hall_0 & 0xff); //(uint8_t)((REG_PIOA_PDSR & 0x00890000) >> 16);
    g_tx_pkt_buf[6] = (uint8_t)(g_hall_1 & 0xff);
    g_tx_pkt_buf[7] = (uint8_t)(g_hall_2 & 0xff);
    //pb = (pb & 0x7) | ((pb & 0x6400) >> 7);
    //g_tx_pkt_buf[6] = (uint8_t)pb;
    rs485_send_packet(0x12, 3);
  }
  else if (pkt_type == 0x13) // motor txrx
  {
    uint8_t en = payload[0];
    uint8_t dir = payload[1];
    uint8_t brake = payload[2];
    int torque[3] = {payload[3], payload[4], payload[5]};

    if (brake)
      PIO_Clear(&pin_m_brake);
    else
      PIO_Set(&pin_m_brake);

    motors_set_all(en, dir, torque);

    *((int32_t *)(g_tx_pkt_buf+ 5)) = g_hall_count_0;
    *((int32_t *)(g_tx_pkt_buf+ 9)) = g_hall_count_1;
    *((int32_t *)(g_tx_pkt_buf+13)) = g_hall_count_2;

    rs485_send_packet(0x13, 12);
  }
  else if (pkt_type == 0x14)
  {
    NVIC_DisableIRQ(PIOA_IRQn);
    NVIC_DisableIRQ(PIOB_IRQn);
    *((int32_t *)(g_tx_pkt_buf+ 5)) = g_hall_count_0;
    *((int32_t *)(g_tx_pkt_buf+ 9)) = g_hall_count_1;
    *((int32_t *)(g_tx_pkt_buf+13)) = g_hall_count_2;
    *((int32_t *)(g_tx_pkt_buf+17)) = g_hall_vel[0];
    *((int32_t *)(g_tx_pkt_buf+21)) = g_hall_vel[1];
    *((int32_t *)(g_tx_pkt_buf+25)) = g_hall_vel[2];
    NVIC_EnableIRQ(PIOA_IRQn);
    NVIC_EnableIRQ(PIOB_IRQn);
    if (payload[0] == 0) // apply encoder offset
    {
      *((int32_t *)(g_tx_pkt_buf+ 5)) -= g_params.encoder_offset[0];
      *((int32_t *)(g_tx_pkt_buf+ 9)) -= g_params.encoder_offset[1];
      *((int32_t *)(g_tx_pkt_buf+13)) -= g_params.encoder_offset[2];
    }
    rs485_send_packet(0x14, 24);
  }
  else if (pkt_type == 0x16) // get number of registered parameters
  {
    *((uint16_t *)(g_tx_pkt_buf+ 5)) = (uint16_t)g_num_registered_params;
    rs485_send_packet(0x16, 2);
  }
  else if (pkt_type == 0x17) // get name of parameter
  {
    uint16_t param_idx;
    param_idx = *((uint16_t *)(payload));
    if (param_idx >= g_num_registered_params)
      rs485_send_packet(0x17, 0);
    else
    {
      uint16_t l = strlen(g_registered_params[param_idx].name);
      if (l > 255)
        l = 255;
      *(g_tx_pkt_buf+5) = (uint8_t)l;
      strncpy((char *)(g_tx_pkt_buf+6),
              g_registered_params[param_idx].name, l);
      rs485_send_packet(0x17, l+1);
    }
  }
  else if (pkt_type == 0x18) // get parameter value
  {
    uint16_t param_idx;
    param_idx = *((uint16_t *)(payload));
    if (param_idx >= g_num_registered_params)
      rs485_send_packet(0x18, 0);
    else
    {
      if (g_registered_params[param_idx].name[0] == 'f')
        *((float *)(g_tx_pkt_buf+5)) =
          *(float *)(g_registered_params[param_idx].val);
      else
        *((uint32_t *)(g_tx_pkt_buf+5)) =
          *(uint32_t *)(g_registered_params[param_idx].val);
      rs485_send_packet(0x18, 4);
    }
  }
  else if (pkt_type == 0x19) // set parameter value
  {
    uint16_t param_idx;
    param_idx = *((uint16_t *)(payload));
    if (param_idx >= g_num_registered_params)
      rs485_send_packet(0x19, 1);
    else
    {
      if (g_registered_params[param_idx].name[0] == 'f')
        *(float *)(g_registered_params[param_idx].val) =
          *((float *)(payload+2));
      else
        *(uint32_t *)(g_registered_params[param_idx].val) =
          *((uint32_t *)(payload+2));
      rs485_send_packet(0x19, 0);
    }
  }
  else if (pkt_type == 0x1a) // commit parameters to flash
  {
    params_save_all_to_flash();
    rs485_send_packet(0x1a, 1);
   }
  else if (pkt_type == 0x1b) // reload parameters
  {
    params_load_all_from_flash();
    rs485_send_packet(0x1b, 0);
  }
  else if (pkt_type == 0x1c)
  {
    params_set_all_default();
    rs485_send_packet(0x1c, 0);
  }
  else if (pkt_type == 0x1d) // set control mode
  {
    enum control_mode_t cm = payload[0];
    if (cm == CM_IDLE)
      control_halt();
    else if (cm == CM_MOTOR_SPACE)
      control_set_motorspace((int16_t *)(payload+1));
    else if (cm == CM_JOINT_SPACE &&
             payload_len == 13)
      control_set_jointspace((float *)(payload+1));
    else if (cm == CM_JOINT_SPACE_FP &&
             payload_len == 7)
      control_set_jointspace_fp((int16_t *)(payload+1));
    else if (cm == CM_JOINT_SPACE_WITH_MAX_EFFORT &&
             payload_len == 16)
      control_set_jointspace_with_max_effort((float *)(payload+1),
                                             payload+13);
    else if (cm == CM_JOINT_SPACE_RELATIVE &&
             payload_len == 16)
      control_set_relative_jointspace((float *)(payload+1), payload+13);
    // avoid generating (potentially colliding) return traffic to this stream.
    //if (payload[0] < (uint8_t)CM_JOINT_SPACE) 
    //  rs485_send_packet(0x1d, 0); // only generate traffic for motor msgs
  }
  else if (pkt_type == 0x1e) // set phalange power
  {
    pb_set_power(payload[0] & 0x01);
    rs485_send_packet(0x1e, 0);
  }
  else if (pkt_type  == 0x1f) // send payload to phalange
  {
    const uint16_t pb_pkt_len = *((uint16_t *)payload);
    const uint16_t max_ms = *((uint16_t *)(payload+2)); 
    volatile uint16_t pkt_load = 0;
    if (pb_pkt_len <= 512)
      pb_send_block(payload+4, pb_pkt_len);
    pb_wait_for_traffic(max_ms, &pkt_load, g_tx_pkt_buf+7);
    *((uint16_t *)(g_tx_pkt_buf+5)) = pkt_load;
    rs485_send_packet(0x1f, pkt_load + 2);
  }
  else if (pkt_type == 0x20) // set phalange polling on/off
  {
    // todo... someday generalize to allow setting the polling frequency
    pb_set_auto_polling(payload[0]);
    rs485_send_packet(0x20, 0);
  }
  else if (pkt_type == 0x21) // read status
  {
    for (int i = 0; i < STATUS_PAYLOAD_LEN; i++)
      g_tx_pkt_buf[5+i] = ((uint8_t *)&g_status)[i];
    rs485_send_packet(0x21, STATUS_PAYLOAD_LEN);
  }
  else if (pkt_type == 0x22) // read temperatures
  {
    // three sources of temperature data: 
    //   I2C temperature probe
    //   MCU internal sensor
    //   accel/mag internal sensor
    *((uint16_t *)(&g_tx_pkt_buf[5])) = g_i2c_sensors_data[0];
    *((uint16_t *)(&g_tx_pkt_buf[7])) = g_adc_data[1];
    *((uint16_t *)(&g_tx_pkt_buf[9])) = g_i2c_sensors_data[7];
    rs485_send_packet(0x22, 6);
  }
  else if (pkt_type == 0x23) // set max effort from mobo
  {
    control_set_max_effort_mobo(payload[0]);
    // don't send response to this packet, since it will come fairly fast
  }
  else if (pkt_type == 0xfa)
  {
    // read hardware version
    *((uint32_t *)(g_tx_pkt_buf + 5)) = g_bl_hw_version;
    rs485_send_packet(0xfa, 4); 
  }
  else if (pkt_type == 0xfd) // announce myself
  {
    // todo: figure out a way to add a random delay here, to avoid collisions
    *((uint16_t *)(g_tx_pkt_buf+ 5)) = g_rs485_address;
    rs485_send_packet(0xfd, 1);
  }
  else if (pkt_type == 0xfe) // reassign address
  {
    if (payload[0] == 0x42 && payload[1] == 0x42 && payload[2] == 0x42)
    {
      // we need to re-write flash page 15, changing just the high 4 bytes
      const uint32_t new_addr = payload[3];
      volatile uint32_t write_ofs = 0;
      const uint32_t page_num = 15;
      __disable_irq();
      for (write_ofs = 0; write_ofs < 252; write_ofs += 4)
      {
        uint32_t write_addr = 0x40000 + 256 * page_num + write_ofs;
        uint32_t read_addr = 0x400000 + 256 * page_num + write_ofs;
        *((uint32_t *)write_addr) = *((uint32_t *)read_addr); // stuff buffer
      }
      *((uint32_t *)(0x40000 + page_num*256 + 252)) = new_addr;
      // TODO: this is terrible and will crash due to single-bank flash...
      EFC->EEFC_FMR &= ~((uint32_t)EEFC_FMR_FRDY); // no interrupt 
      /*
      EFC->EEFC_FCR = EEFC_FCR_FKEY(0x5A) |
                      EEFC_FCR_FCMD(EFC_FCMD_EWP) | // erase and write page
                      EEFC_FCR_FARG(page_num);
      */
      while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // spin...
      __enable_irq();
      g_rs485_address = new_addr;
      rs485_send_packet(0xfe, 0);
    }
  }
}

void rs485_handle_byte(uint8_t b)
{
  static enum parser_state_t { ST_IDLE = 0, ST_HEADER = 1, ST_ADDRESS = 2,
                               ST_LEN_1 = 3, ST_LEN_2 = 4, ST_TYPE = 5,
                               ST_DATA = 6, ST_CRC_1 = 7, ST_CRC_2 = 8}
                             state = ST_IDLE;
  static uint8_t  pkt_addr = 0;
  static uint16_t pkt_len  = 0;
  static uint8_t  pkt_type = 0;
  static uint16_t pkt_write_idx = 0;
  static uint8_t  pkt_data[COMMS_MAX_PACKET_LENGTH];
  static uint16_t pkt_crc = 0;

  switch(state)
  {
    case ST_IDLE:
      if (b == 0x42)
        state = ST_HEADER;
      break;
    case ST_HEADER:
      pkt_addr = b;
      if (pkt_addr != 0x42)
        state = ST_LEN_1;
      break;
    case ST_LEN_1:
      pkt_len = b;
      state = ST_LEN_2;
      break;
    case ST_LEN_2:
      pkt_len |= ((uint16_t)b << 8);
      if (pkt_len > 280)
        state = ST_IDLE; // bogus length.
      else
        state = ST_TYPE;
      break;
    case ST_TYPE:
      pkt_type = b;
      pkt_write_idx = 0;
      if (pkt_len > 0)
        state = ST_DATA;
      else
        state = ST_CRC_1;
      break;
    case ST_DATA:
      if (pkt_write_idx < COMMS_MAX_PACKET_LENGTH)
        pkt_data[pkt_write_idx++] = b;
      if (pkt_write_idx >= pkt_len)
        state = ST_CRC_1;
      break;
    case ST_CRC_1:
      pkt_crc = b;
      state = ST_CRC_2;
      break;
    case ST_CRC_2:
      pkt_crc |= ((uint16_t)b << 8);
      rs485_process_packet(pkt_addr, pkt_len, pkt_type, pkt_data, pkt_crc);
      state = ST_IDLE;
      break;
    default:
      state = ST_IDLE;
      break;
  }
}

void rs485_irq()
{
  // this has to be a _really_ fast ISR!
  if (USART0->US_CSR & US_CSR_RXRDY)
  {
    g_rx_buf[g_rx_buf_writepos++] = USART0->US_RHR;
    if (g_rx_buf_writepos >= COMMS_RX_BUF_LEN)
      g_rx_buf_writepos = 0;
  }
}

void poweron_5v()
{
  PIO_Configure(&pin_5v_reg, 1);
  PIO_Set(&pin_5v_reg);
}

void systick_irq()
{
  control_systick();
  pb_systick();
  i2c_sensors_systick();
  adc_systick();
}

int main(void)
{
  //volatile uint32_t i;
  //for (i = 0; i < 1000000; i++) { }
  WDT_Disable(WDT);
  PMC_EnablePeripheral(ID_PIOA);
  PMC_EnablePeripheral(ID_PIOB);
  PIO_Configure(&pin_led, 1);
  init_clocks();
  gpio_led(false);
  //console_init();
  rs485_init();
  status_init();
  imu_init();
  halls_init();
  poweron_5v();
  motors_init();
  
  /*
  while (1)
  {
    PIO_Set(&pin_m0_en);
    PIO_Set(&pin_m1_en);
    PIO_Set(&pin_m2_en);
  }
  */
  
  pb_init(); 
  adc_init();
  params_load_all_from_flash();
  control_init();
  i2c_sensors_init();
  __enable_irq();

  while (1)
  {
    if (g_rx_buf_writepos != g_rx_buf_readpos)  // chase the USART ringbuffer
    {
      rs485_handle_byte(g_rx_buf[g_rx_buf_readpos++]);
      if (g_rx_buf_readpos >= COMMS_RX_BUF_LEN)
        g_rx_buf_readpos = 0;
    }
    pb_idle();
    i2c_sensors_idle();
    adc_idle();
  }
}

///////////////////////////////////////////////////////////////////////////
// libc stubs... we don't have an OS, so just put crap here.
//////////////////////////////////////////////////////////////////////////

extern int _end;
extern caddr_t _sbrk(int incr)
{
  static unsigned char *heap = NULL ;
  unsigned char *prev_heap ;
  if ( heap == NULL )
    heap = (unsigned char *)&_end ;
  prev_heap = heap;
  heap += incr ;
  return (caddr_t) prev_heap ;
}
extern int _kill(int pid, int sig) { return -1; }
extern void _exit(int status) { }
int _getpid() { return 1; }
extern int _write(int fd, const void *buf, size_t count)
{
  return count;
}
int _close(int fd) { return -1; }
int _fstat(int fd, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}
int _isatty(int fd) { return 1; }
off_t _lseek(int fd, off_t offset, int whence) { return 0; }
ssize_t _read(int fd, void *buf, size_t count) { return 0; }

struct __FILE { int handle; };
FILE __stdout;
FILE __stderr;
int fputc(int ch, FILE *f)
{
  //console_send_string("fputc\r\n");
  return 0;
}
void _ttywrch(int ch)
{
  //console_send_string("ttywrch\r\n");
}

/////////////////////////////////////////////////////////////
/// graveyard
#if 0
void console_init()
{
  PIO_Configure(&pin_aux_rx, 1);
  PIO_Configure(&pin_aux_tx, 1);
  PIO_Configure(&pin_aux0, 1);
  PIO_Configure(&pin_aux1, 1);
  PMC_EnablePeripheral(ID_USART1);
  USART1->US_IDR  = 0xffffffff; // no interrupts plz
  
  USART1->US_CR   = US_CR_RSTRX | US_CR_RSTTX | 
                    US_CR_RXDIS | US_CR_TXDIS; // reset uart
  USART1->US_MR   = US_MR_CHRL_8_BIT | US_MR_PAR_NO; // 8 bit, no parity, 1 stop
  USART1->US_BRGR = (F_CPU / 1000000) / 16;
  USART1->US_CR   = UART_CR_TXEN | UART_CR_RXEN; // eanble TX and RX
  //USART1->US_IER  = UART_IER_RXRDY; // enable RX interrupt
  //NVIC_EnableIRQ(UART1_IRQn);
  g_console_init_complete = 1;
}

void console_send_block(uint8_t *block, uint32_t len)
{
  while ((USART1->US_CSR & US_CSR_TXEMPTY) == 0) { }
  for (uint32_t i = 0; i < len; i++)
  {
    USART1->US_THR = block[i];
    while ((USART1->US_CSR & US_CSR_TXEMPTY) == 0) { }
  }
}
#endif

