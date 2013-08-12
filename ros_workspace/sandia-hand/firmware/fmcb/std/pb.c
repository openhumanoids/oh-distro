#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "pb.h"
#include "comms.h"
#include "pins.h"
#include "status.h"
#include "control.h"
#include "halls.h"
#include "params.h"
#include "gpio.h"
#include "adc.h"
#include "i2c_sensors.h"

static volatile uint8_t g_pb_rx_buf[COMMS_RX_BUF_LEN];
static volatile uint8_t g_pb_tx_buf[COMMS_MAX_PACKET_LENGTH];
static volatile unsigned g_pb_rx_buf_writepos = 0, g_pb_rx_buf_readpos = 0;
static volatile uint32_t g_pb_txrx_ms = 0;
static void pb_handle_byte(const uint8_t b);
static void pb_process_packet(const uint8_t pkt_addr, 
                              const uint16_t payload_len,
                              const uint8_t pkt_type, 
                              const uint8_t *payload,
                              const uint16_t pkt_crc);
static void pb_send_packet(const uint8_t addr, 
                           const uint8_t pkt_type, 
                           const uint16_t payload_len);
#define PB_PS_IDLE         0
#define PB_PS_PP_TACTILE   1
#define PB_PS_PP_STRAIN    2
#define PB_PS_PP_IMU_ON    3
#define PB_PS_PP_IMU_POLL  4
#define PB_PS_PP_IMU_OFF   5
#define PB_PS_PP_TEMP      6
#define PB_PS_DP_TACTILE   7
#define PB_PS_DP_IMU_ON    8
#define PB_PS_DP_IMU_POLL  9
#define PB_PS_DP_IMU_OFF  10
#define PB_PS_DP_TEMP     11
#define PB_PS_POLL_REQ   0x80
static int g_pb_poll_state = PB_PS_IDLE;
static int g_pb_systick_count = 0;
static bool g_pb_auto_polling = false; 
static bool g_pb_auto_drain = true;
static volatile uint32_t g_pb_tc0_ovf_count = 0;

#define PB_PP_ADDR 1
#define PB_DP_ADDR 2

typedef enum { PB_CB_IDLE, PB_CB_OVER_LIMIT, PB_CB_TRIPPED } pb_cb_state_t;
static pb_cb_state_t g_pb_cb_state = PB_CB_IDLE;

inline void pb_systick_disable()
{
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

inline void pb_systick_enable()
{
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

void pb_init()
{
  PMC_EnablePeripheral(ID_TC0);
  TC0->TC_QIDR = 0xffffffff; // no quadrature interrupts plz
  TC0->TC_CHANNEL[0].TC_IDR = 0xffffffff; // no timer interrupts
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_COVFS; // enable overflow interrupt
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | // 64 / 128 = 500khz
                              TC_CMR_WAVE; // waveform generation mode
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;
  NVIC_SetPriority(TC0_IRQn, 4);
  NVIC_EnableIRQ(TC0_IRQn);
  PMC_EnablePeripheral(ID_USART1);
  PIO_Configure(&pin_phal_pwr, 1);
  PIO_Configure(&pin_phal_de, 1);
  PIO_Configure(&pin_phal_di, 1);
  PIO_Configure(&pin_phal_ro, 1);
  USART1->US_CR = US_CR_RSTRX | UART_CR_RSTTX | 
                  US_CR_RXDIS | UART_CR_TXDIS; // reset usart
  USART1->US_IDR = 0xffffffff; // disable all interrupts
  USART1->US_MR = US_MR_CHRL_8_BIT | US_MR_PAR_NO | 
                  US_MR_MAN | US_MR_OVER; // 8N1, normal mode
  USART1->US_BRGR = F_CPU / 2000000 / 16;
  USART1->US_CR = US_CR_TXEN | US_CR_RXEN; // eanble TX and RX
  USART1->US_MAN = US_MAN_TX_PL(1) | US_MAN_TX_PP_ALL_ONE |
                   US_MAN_RX_PL(1) | US_MAN_RX_PP_ALL_ONE | 
                   US_MAN_RX_MPOL | US_MAN_TX_MPOL |
                   US_MAN_DRIFT | US_MAN_STUCKTO1;
  USART1->US_IER = UART_IER_RXRDY; // enable RX interrupt
  NVIC_SetPriority(USART1_IRQn, 2);
  NVIC_EnableIRQ(USART1_IRQn);
  PIO_Clear(&pin_phal_de);
}

void pb_send_block(uint8_t *block, uint32_t len)
{
  /*
  NVIC_DisableIRQ(TWI0_IRQn);
  NVIC_DisableIRQ(TC0_IRQn);
  */
  //__disable_irq(); // must not be interrupted during this burst? think...
  NVIC_DisableIRQ(ADC_IRQn);
  pb_systick_disable();
  USART1->US_CR |= US_CR_RXDIS; // avoid loopback
  while ((USART1->US_CSR & US_CSR_TXRDY) == 0) { }
  PIO_Set(&pin_phal_de);
  //for (volatile int d = 0; d < 2; d++) { }
  for (uint32_t i = 0; i < len; i++)
  {
    USART1->US_THR = block[i];
    while ((USART1->US_CSR & US_CSR_TXRDY) == 0) { }
  }
  while ((USART1->US_CSR & US_CSR_TXEMPTY) == 0) { }
  //for (volatile int d = 0; d < 1; d++) { }
  PIO_Clear(&pin_phal_de);
  USART1->US_CR &= ~US_CR_RXDIS;
  USART1->US_CR |= US_CR_RXEN;
  if (USART1->US_CSR & US_CSR_RXRDY)
    USART1->US_RHR; // clear rx buffer
  pb_systick_enable();
  NVIC_EnableIRQ(ADC_IRQn);
  //__enable_irq();
}

void pb_usart1_irq()
{
  if (USART1->US_CSR & US_CSR_RXRDY) // this has to be a _really_ fast ISR!
  {
    g_pb_rx_buf[g_pb_rx_buf_writepos] = USART1->US_RHR;
    if (++g_pb_rx_buf_writepos >= COMMS_RX_BUF_LEN)
      g_pb_rx_buf_writepos = 0;
  }
}

void pb_handle_byte(const uint8_t b)
{
  // this is so bad. this code is copied into like 6 places. must generalize
  // it and stuff it in a library someday.
  static enum parser_state_t { ST_IDLE = 0, ST_HEADER = 1, ST_ADDRESS = 2,
                               ST_LEN_1 = 3, ST_LEN_2 = 4, ST_TYPE = 5,
                               ST_DATA = 6, ST_CRC_1 = 7, ST_CRC_2 = 8}
                             s_pb_state = ST_IDLE;
  static uint8_t  s_pb_pkt_addr = 0;
  static uint16_t s_pb_pkt_len  = 0;
  static uint8_t  s_pb_pkt_type = 0;
  static uint16_t s_pb_pkt_write_idx = 0;
  static uint8_t  s_pb_pkt_data[COMMS_MAX_PACKET_LENGTH];
  static uint16_t s_pb_pkt_crc = 0;

  switch(s_pb_state)
  {
    case ST_IDLE:
      if (b == 0x42)
        s_pb_state = ST_HEADER;
      break;
    case ST_HEADER:
      s_pb_pkt_addr = b;
      if (s_pb_pkt_addr != 0x42)
        s_pb_state = ST_LEN_1;
      break;
    case ST_LEN_1:
      s_pb_pkt_len = b;
      s_pb_state = ST_LEN_2;
      break;
    case ST_LEN_2:
      s_pb_pkt_len |= ((uint16_t)b << 8);
      if (s_pb_pkt_len > 280)
        s_pb_state = ST_IDLE; // bogus length.
      else
        s_pb_state = ST_TYPE;
      break;
    case ST_TYPE:
      s_pb_pkt_type = b;
      s_pb_pkt_write_idx = 0;
      if (s_pb_pkt_len > 0)
        s_pb_state = ST_DATA;
      else
        s_pb_state = ST_CRC_1;
      break;
    case ST_DATA:
      if (s_pb_pkt_write_idx < COMMS_MAX_PACKET_LENGTH)
        s_pb_pkt_data[s_pb_pkt_write_idx++] = b;
      if (s_pb_pkt_write_idx >= s_pb_pkt_len)
        s_pb_state = ST_CRC_1;
      break;
    case ST_CRC_1:
      s_pb_pkt_crc = b;
      s_pb_state = ST_CRC_2;
      break;
    case ST_CRC_2:
      s_pb_pkt_crc |= ((uint16_t)b << 8);
      pb_process_packet(s_pb_pkt_addr, s_pb_pkt_len, s_pb_pkt_type, 
                        s_pb_pkt_data, s_pb_pkt_crc);
      s_pb_state = ST_IDLE;
      break;
    default:
      s_pb_state = ST_IDLE;
      break;
  }
}

void pb_process_packet(const uint8_t pkt_addr, 
                       const uint16_t payload_len,
                       const uint8_t pkt_type, 
                       const uint8_t *payload,
                       const uint16_t pkt_crc)
{
  uint16_t crc = 0;
  uint8_t d, crc_highbit;

  if (pkt_addr != 0 && pkt_addr != 0xff)  // finger mcb is zero on this bus
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
    return; // packet checksum mismatch
  //gpio_led(true);
  if (pkt_type == 0x10) // tactile query
  { // store results in our status buffer
    if (g_pb_poll_state == PB_PS_PP_TACTILE)
    {
      g_status.pp_tactile_time = 0; // TODO
      for (int i = 0; i < PP_NUM_TAXELS*2; i++) 
        *(((uint8_t *)g_status.pp_tactile) + i) = payload[i];
      for (int i = 0; i < STATUS_IMU_LEN*2; i++)
        *(((uint8_t *)g_status.pp_imu_data)+i) = payload[PP_NUM_TAXELS*2 + i];
    }
    else if (g_pb_poll_state == PB_PS_DP_TACTILE)
    {
      g_status.dp_tactile_time = 0; // TODO
      for (int i = 0; i < DP_NUM_TAXELS*2; i++)
        *(((uint8_t *)g_status.dp_tactile) + i) = payload[i];
      for (int i = 0; i < STATUS_IMU_LEN*2; i++)
        *(((uint8_t *)g_status.dp_imu_data)+i) = payload[DP_NUM_TAXELS*2 + i];
    }
  }
  else if (pkt_type == 0x12) // imu query
  { // store results in our status buffer...
    if (g_pb_poll_state == PB_PS_PP_IMU_POLL)
      for (int i = 0; i < STATUS_IMU_LEN; i++)
        g_status.pp_imu_data[i] = *(((uint16_t *)payload) + i);
    else if (g_pb_poll_state == PB_PS_DP_IMU_POLL)
      for (int i = 0; i < STATUS_IMU_LEN; i++)
        g_status.dp_imu_data[i] = *(((uint16_t *)payload) + i);
  }
  else if (pkt_type == 0x15) // read adc
  {
    if (g_pb_poll_state == PB_PS_PP_STRAIN)
      g_status.pp_strain = *((uint32_t *)payload);
    else if (g_pb_poll_state == PB_PS_PP_TEMP)
      for (int i = 0; i < 3; i++)
        g_status.pp_temp[i] = payload[i];
  }
  g_pb_poll_state = PB_PS_IDLE; // assume this was our response, bus is clear
  //gpio_led(false);
}

void pb_wait_for_traffic(uint16_t max_ms,
                         volatile uint16_t *num_bytes_recv, 
                         volatile uint8_t *bytes_recv)
{
  pb_systick_disable();
  g_pb_txrx_ms = 0;
  pb_systick_enable();
  g_pb_auto_drain = false; // we'll drain it ourselves...
  while (g_pb_txrx_ms < max_ms)
  {
    // wait for incoming data, package it up. we'll be interrupted here, 
    // but that's fine.
    while (g_pb_rx_buf_writepos != g_pb_rx_buf_readpos)
    {
      (*num_bytes_recv)++;
      *bytes_recv = g_pb_rx_buf[g_pb_rx_buf_readpos];
      bytes_recv++; // can probably combine this line with previous somehow?
      // also run this byte through our local parser, to maintain state
      pb_handle_byte(g_pb_rx_buf[g_pb_rx_buf_readpos]);
      if (++g_pb_rx_buf_readpos >= COMMS_RX_BUF_LEN)
        g_pb_rx_buf_readpos = 0;
    }
  }
  g_pb_auto_drain = true; // empty PB buffer in this module's idle task
}

void pb_circuit_breaker(const uint16_t current_raw_adc)
{
  const float pb_amps = current_raw_adc * 3.3f / 1024.0f / 2.7f;
  const float PB_CB_CURRENT_LIMIT_AMPS = 0.2f;
  const uint32_t PB_CB_MAX_OVER_MS = 10;
  static uint32_t s_pb_over_limit_ms = 0;
  switch (g_pb_cb_state)
  {
    case PB_CB_IDLE:
      if (pb_amps > PB_CB_CURRENT_LIMIT_AMPS)
      {
        g_pb_cb_state = PB_CB_OVER_LIMIT;
        s_pb_over_limit_ms = 0;
      }
      break;
    case PB_CB_OVER_LIMIT:
      if (pb_amps > PB_CB_CURRENT_LIMIT_AMPS)
      {
        s_pb_over_limit_ms++;
        if (s_pb_over_limit_ms > PB_CB_MAX_OVER_MS)
        {
          // it's been over the limit for too long; need to shut it down
          pb_set_power(false);
          g_pb_cb_state = PB_CB_TRIPPED;
        }
      }
      else
        g_pb_cb_state = PB_CB_IDLE;
      break;
    case PB_CB_TRIPPED:
      break;
    default:  
      break; // shouldn't ever get here.
  }
}

void pb_systick()
{
  g_pb_txrx_ms++;
  g_pb_systick_count++; // keep it in [0..999], wraps each second
  if (g_pb_systick_count >= 1000)
    g_pb_systick_count = 0;
  const int pb_subclock = g_pb_systick_count % 10;
  pb_circuit_breaker(g_adc_data[2]);
  if (pb_subclock % 5 == 0)
  {
    // snapshot encoder targets and values
    g_status.fmcb_time = (g_pb_tc0_ovf_count << 17) + 
                         (TC0->TC_CHANNEL[0].TC_CV << 1);  // microseconds
    for (int i = 0; i < 3; i++)
    {
      g_status.fmcb_hall_tgt[i] = g_control_hall_tgt[i];
      g_status.fmcb_effort[i] = g_control_effort[i];
    }
    g_status.fmcb_hall_pos[0] = g_hall_count_0 - g_params.encoder_offset[0];
    g_status.fmcb_hall_pos[1] = g_hall_count_1 - g_params.encoder_offset[1];
    g_status.fmcb_hall_pos[2] = g_hall_count_2 - g_params.encoder_offset[2];
    for (int i = 0; i < 3; i++)
      g_status.fmcb_imu_data[i] = g_i2c_sensors_data[i+1]; // accel
    for (int i = 4; i < 6; i++)
      g_status.fmcb_imu_data[i] = g_i2c_sensors_data[i+1]; // mag
    g_status.fmcb_temp[0] = g_i2c_sensors_data[0];
    g_status.fmcb_temp[1] = g_i2c_sensors_data[7];
    g_status.fmcb_temp[2] = g_adc_data[1];
    g_status.fmcb_pb_current = (g_pb_cb_state == PB_CB_TRIPPED ? 
                                0xffff : g_adc_data[2]); 
    g_status.fmcb_voltage = g_adc_data[0];
  }
  if (!g_pb_auto_polling)
    return; 
  // poll phalange sensors on a 10ms cycle
  switch (pb_subclock)
  {
    case 0:  g_pb_poll_state = PB_PS_PP_TACTILE | PB_PS_POLL_REQ; break;
    case 1:  g_pb_poll_state = PB_PS_DP_TACTILE | PB_PS_POLL_REQ; break;
    case 2:  g_pb_poll_state = PB_PS_PP_STRAIN  | PB_PS_POLL_REQ; break;
    default: g_pb_poll_state = PB_PS_IDLE; break;
  }
  // poll inertial and temperature sensors once per second
  if (g_pb_systick_count >= 983) 
  {
    g_pb_poll_state = PB_PS_IDLE; // hiccup the tactile stream 
    switch (g_pb_systick_count)
    {
      case 983: g_pb_poll_state = PB_PS_PP_TEMP     | PB_PS_POLL_REQ; break;
      case 984: g_pb_poll_state = PB_PS_DP_TEMP     | PB_PS_POLL_REQ; break;
      case 985: g_pb_poll_state = PB_PS_PP_IMU_ON   | PB_PS_POLL_REQ; break;
      case 986: g_pb_poll_state = PB_PS_DP_IMU_ON   | PB_PS_POLL_REQ; break;
      case 996: g_pb_poll_state = PB_PS_PP_IMU_POLL | PB_PS_POLL_REQ; break;
      case 997: g_pb_poll_state = PB_PS_DP_IMU_POLL | PB_PS_POLL_REQ; break;
      case 998: g_pb_poll_state = PB_PS_PP_IMU_OFF  | PB_PS_POLL_REQ; break;
      case 999: g_pb_poll_state = PB_PS_DP_IMU_OFF  | PB_PS_POLL_REQ; break;
      default: break;
    }
  }
}

void pb_idle()
{
  if (!g_pb_auto_drain)
    return;
  // chase our RX ringbuffer
  while (g_pb_rx_buf_writepos != g_pb_rx_buf_readpos)
  {
    pb_handle_byte(g_pb_rx_buf[g_pb_rx_buf_readpos]);
    if (++g_pb_rx_buf_readpos >= COMMS_RX_BUF_LEN)
      g_pb_rx_buf_readpos = 0;
  }
  if (g_pb_poll_state & PB_PS_POLL_REQ)
  {
    g_pb_poll_state &= ~PB_PS_POLL_REQ;
    switch (g_pb_poll_state)
    {
      case PB_PS_PP_TACTILE:
        pb_send_packet(PB_PP_ADDR, 0x10, 0);  
        break;
      case PB_PS_DP_TACTILE:
        pb_send_packet(PB_DP_ADDR, 0x10, 0);  
        break;
      case PB_PS_PP_IMU_ON:
        g_pb_tx_buf[5] = 1; // imu on plz
        pb_send_packet(PB_PP_ADDR, 0x11, 1);
        break;
      case PB_PS_DP_IMU_ON:
        g_pb_tx_buf[5] = 1; // imu on plz
        pb_send_packet(PB_DP_ADDR, 0x11, 1);
        break;
      case PB_PS_PP_IMU_POLL:
        pb_send_packet(PB_PP_ADDR, 0x12, 0);
        break;
      case PB_PS_DP_IMU_POLL:
        pb_send_packet(PB_DP_ADDR, 0x12, 0);
        break;
      case PB_PS_PP_IMU_OFF:
        g_pb_tx_buf[5] = 0; // imu off plz
        pb_send_packet(PB_PP_ADDR, 0x11, 1);
        break;
      case PB_PS_DP_IMU_OFF:
        g_pb_tx_buf[5] = 0; // imu off plz
        pb_send_packet(PB_DP_ADDR, 0x11, 1);
        break;
      case PB_PS_PP_STRAIN:
        break;
      /*
      case PB_PS_PP_TEMP:
        break;
      case PB_PS_DP_TEMP:
        break;
      case PB_PS_IDLE: 
      */
      default:
        break;
    }
  }
}

void pb_set_power(bool on)
{
  if (on)
  {
    g_pb_cb_state = PB_CB_IDLE;
    PIO_Set(&pin_phal_pwr);
  }
  else
  {
    g_pb_cb_state = PB_CB_IDLE;
    PIO_Clear(&pin_phal_pwr);
  }
}

void pb_send_packet(const uint8_t addr, 
                    const uint8_t pkt_type, 
                    const uint16_t payload_len)
{
  uint16_t crc = 0, i, d, crc_highbit;
  g_pb_tx_buf[0] = 0x42;
  g_pb_tx_buf[1] = addr; // 1 = proximal phalange, 2 = distal phalange
  g_pb_tx_buf[2] = payload_len & 0xff;
  g_pb_tx_buf[3] = (payload_len >> 8) & 0xff;
  g_pb_tx_buf[4] = pkt_type;
  for (i = 0; i < payload_len+5; i++)
  {
    d = g_pb_tx_buf[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc_highbit = (crc >> 8) & 0x80;
      crc <<= 1;
      if ((d & 0x80) ^ crc_highbit)
        crc ^= 0x1021; // CRC-16 CCITT polynomial
      d <<= 1;
    }
  }
  g_pb_tx_buf[5+payload_len] = crc & 0xff;
  g_pb_tx_buf[6+payload_len] = (crc >> 8) & 0xff;
  pb_send_block((uint8_t *)g_pb_tx_buf, 7+payload_len);
}

void pb_set_auto_polling(bool on)
{
  g_pb_auto_polling = on;
}

void pb_tc0_irq()
{
  g_pb_tc0_ovf_count++; 
  TC0->TC_CHANNEL[0].TC_SR; // dummy read
}

