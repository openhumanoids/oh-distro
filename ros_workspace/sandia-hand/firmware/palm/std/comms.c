#include "string.h"
#include "sam3s/chip.h"
#include "sam3s/pmc.h"
#include "pins.h"
#include "comms.h"
#include "tactile.h"
#include "imu.h"
#include "state.h"

void comms_send_packet(uint8_t pkt_type, uint16_t payload_len);
void comms_send_block(uint8_t *block, uint32_t len);
void comms_process_packet(uint8_t pkt_addr, uint16_t payload_len,
                          uint8_t pkt_type, uint8_t *payload,
                          uint16_t pkt_crc);

static volatile uint8_t g_tx_pkt_buf[1024];
static int g_comms_rs485_address = 0xfe; // bogus 
static uint32_t g_bl_hw_version = 0;
#define RX_BUF_LEN 512
static volatile uint8_t g_rx_buf[RX_BUF_LEN];
static volatile unsigned g_rx_buf_writepos = 0, g_rx_buf_readpos = 0;
static volatile unsigned g_comms_rx_pkt_timer = 0;
static volatile uint32_t last_state_send_time = 0; 
#define COMMS_RX_PKT_TIMEOUT_MS 10
static volatile Usart *rs485_usart = USART0;

void comms_init()
{
  g_comms_rs485_address = *((uint32_t *)0x0401ffc);
  g_bl_hw_version = *((uint32_t *)0x0401ff8); // magic, defined in bootloader
  if (((g_bl_hw_version >> 16) & 0xffff) != 0xbeef) // check for magic bytes
    g_bl_hw_version = 0; // undefined
  else
    g_bl_hw_version &= 0xffff; // keep useful lower 16 bits
  const char hand = (char)((g_bl_hw_version >> 8) & 0xff);
  if (hand == 'L')
    rs485_usart = USART1;
  else
    rs485_usart = USART0;
  PMC_EnablePeripheral(hand == 'L' ? ID_USART1 : ID_USART0);
  PIO_Configure(&pin_rs485_de, 1);
  PIO_Configure(&pin_rs485_di, 1);
  PIO_Configure(&pin_rs485_ro, 1);
  rs485_usart->US_CR = US_CR_RSTRX | UART_CR_RSTTX | 
                       US_CR_RXDIS | UART_CR_TXDIS; // reset usart
  rs485_usart->US_IDR = 0xffffffff; // disable all interrupts
  rs485_usart->US_MR = US_MR_CHRL_8_BIT | US_MR_PAR_NO; 
  rs485_usart->US_BRGR = F_CPU / 2000000 / 16; 
  rs485_usart->US_CR = UART_CR_TXEN | UART_CR_RXEN; // eanble TX and RX
  rs485_usart->US_IER = UART_IER_RXRDY; // enable RX interrupt
  const IRQn_Type rs485_irqn = (hand == 'L' ? USART1_IRQn : USART0_IRQn);
  NVIC_SetPriority(rs485_irqn, 1);
  NVIC_EnableIRQ(rs485_irqn);
}

void comms_send_block(uint8_t *block, uint32_t len)
{
  volatile uint32_t d = 0;
  //volatile uint8_t dummy_block[10] = {10, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  //volatile uint32_t dummy_block_len = 10;
  //block = dummy_block;
  //len = dummy_block_len;
  /*
  printf("sending: ");
  for (int i = 0; i < len; i++)
    printf("%02x ", block[i]);
  printf("\r\n");
  */
  rs485_usart->US_CR |= US_CR_RXDIS;
  //for (d = 0; d < 1000; d++) { }
  while ((rs485_usart->US_CSR & US_CSR_TXRDY) == 0) { }
  PIO_Set(&pin_rs485_de);
  //for (d = 0; d < 2; d++) { }
  for (uint32_t i = 0; i < len; i++)
  {
    rs485_usart->US_THR = block[i];
    while ((rs485_usart->US_CSR & US_CSR_TXRDY) == 0) { }
  }
  while ((rs485_usart->US_CSR & US_CSR_TXEMPTY) == 0) { }
  for (d = 0; d < 1; d++) { }
  PIO_Clear(&pin_rs485_de);
  //for (d = 0; d < 1000; d++) { }
  rs485_usart->US_CR &= ~US_CR_RXDIS;
  rs485_usart->US_CR |= US_CR_RXEN;
  if (rs485_usart->US_CSR & US_CSR_RXRDY)
    rs485_usart->US_RHR; // flush incoming buffer in case tx put stuff there
}

void comms_send_packet(uint8_t pkt_type, uint16_t payload_len)
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
  comms_send_block((uint8_t *)g_tx_pkt_buf, 7+payload_len);
}

void comms_handle_byte(uint8_t b)
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

  if (g_comms_rx_pkt_timer >= COMMS_RX_PKT_TIMEOUT_MS)
    state = ST_IDLE; // assume the packet is broken.
  //printf("%x\r\n", (int)state); //, state = %d\r\n", b, (int)state);

  switch(state)
  {
    case ST_IDLE:
      if (b == 0x42)
      {
        //led(true);
        state = ST_HEADER;
        g_comms_rx_pkt_timer = 0;
      }
      break;
    case ST_HEADER:
      pkt_addr = b;
      if (pkt_addr == 0x42)
      {
#if 0
        io_led(true);
#endif
      }
      else
        state = ST_LEN_1;
      break;
    case ST_LEN_1:
      pkt_len = b;
      state = ST_LEN_2;
      break;
    case ST_LEN_2:
      pkt_len |= ((uint16_t)b << 8);
      if (pkt_len > 280)
      {
#if 0
        io_led(true);
#endif
        state = ST_IDLE;
      }
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
      comms_process_packet(pkt_addr, pkt_len, pkt_type, pkt_data, pkt_crc);
      state = ST_IDLE;
      break;
    default:
      state = ST_IDLE;
      break;
  }
}

void comms_irq()
{
  // this has to be a _really_ fast ISR!
  if (rs485_usart->US_CSR & US_CSR_RXRDY)
  {
    volatile uint8_t b = rs485_usart->US_RHR;
    g_rx_buf[g_rx_buf_writepos++] = b; 
    if (g_rx_buf_writepos >= RX_BUF_LEN)
      g_rx_buf_writepos = 0;
  }
}

void comms_systick()
{
  g_comms_rx_pkt_timer++;
}

void comms_idle()
{
  while (g_rx_buf_writepos != g_rx_buf_readpos)
  {
    //printf("rx %02x\r\n", g_rx_buf[g_rx_buf_readpos]);
    comms_handle_byte(g_rx_buf[g_rx_buf_readpos++]);
    if (g_rx_buf_readpos >= RX_BUF_LEN)
      g_rx_buf_readpos = 0;
  }
}

void comms_process_packet(uint8_t pkt_addr, uint16_t payload_len,
                          uint8_t pkt_type, uint8_t *payload,
                          uint16_t pkt_crc)
{
  //printf("%d\r\n", pkt_addr);
  uint16_t crc = 0;
  uint8_t d, crc_highbit;

  if (pkt_addr != g_comms_rs485_address && pkt_addr != 0xff) 
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
    //printf("packet type %d address %d rejected due to checksum error\r\n",
    //       pkt_type, pkt_addr);
    return;
  }
  if (pkt_type == 0x01)     // ping packet. respond with ping back.
  {
    for (volatile int i = 0; i < 10; i++) { }
    comms_send_packet(0x01, 0);
  }
  else if (pkt_type == 0x09) // reset 
  {
    comms_send_packet(9, 0); // ack the request
    RSTC->RSTC_CR = RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;
  }
  else if (pkt_type == 0x10)
  {
    __disable_irq(); // todo: something less drastic. just prevent systicks.
    for (int i = 0; i < TACTILE_NUM_TAXELS; i++)
      *((uint16_t *)(&g_tx_pkt_buf[5+i*2])) = g_tactile_last_scan[i];
    __enable_irq();
    comms_send_packet(0x10, TACTILE_NUM_TAXELS*2);
  }
  else if (pkt_type == 0x11) // subsystem power
  {
    if (payload[0] & 0x01)
      imu_power(1);
    else
      imu_power(0);
    comms_send_packet(0x11, 0);
  }
  else if (pkt_type == 0x12) // poll imu
  {
    for (uint8_t i = 0; i < 12; i++)
      g_tx_pkt_buf[5+i] = ((uint8_t *)(g_imu_data))[i];
    comms_send_packet(0x12, 12);
  }
  else if (pkt_type == 0x21) // poll state buffer
  {
    //if (last_state_send_time == g_state.palm_time)
    //  return; // don't send same tactile scan more than once
    last_state_send_time = g_state.palm_time;
    for (int i = 0; i < sizeof(palm_state_t); i++)
      g_tx_pkt_buf[5+i] = ((uint8_t *)&g_state)[i]; // ugly
    comms_send_packet(0x21, sizeof(palm_state_t));
  }
  else if (pkt_type == 0xfa) // read hardware version
  {
    *((uint32_t *)(g_tx_pkt_buf + 5)) = g_bl_hw_version;
    comms_send_packet(0xfa, 4);
  }
  //else
  //  printf("unknown pkt type: 0x%02x\r\n", pkt_type);


#if 0
  else if (pkt_type == 0x10) // accel/mag read
  {
    const uint8_t device = payload[0];
    for (uint8_t i = 0; i < payload_len-1; i++)
      g_tx_pkt_buf[i+5] = imu_reg_read(device, payload[i+1]); 
    comms_send_packet(0x10, payload_len-1);
  }
  else if (pkt_type == 0x11) // accel/mag write
  {
    const uint8_t device = payload[0];
    const uint8_t num_regs = (payload_len-1)/2;
    for (uint8_t i = 0; i < num_regs; i++)
    {
      uint8_t reg_addr    = payload[i*2+1];
      uint8_t reg_payload = payload[i*2+2];
      imu_reg_write(device, reg_addr, reg_payload);
    }
    comms_send_packet(0x11, 0);
  }
  else if (pkt_type == 0x15) // read adc
  {
    *((int32_t *)(g_tx_pkt_buf+5)) = adc_read_busywait(payload[0]);
    comms_send_packet(0x15, 4);
  }
  else if (pkt_type == 0x16) // get number of registered parameters
  {
    *((uint16_t *)(g_tx_pkt_buf+ 5)) = (uint16_t)g_num_registered_params;
    comms_send_packet(0x16, 4);
  }
  else if (pkt_type == 0x17) // get name of parameter
  {
    uint16_t param_idx;
    param_idx = *((uint16_t *)(payload));
    if (param_idx >= g_num_registered_params)
      comms_send_packet(0x17, 0);
    else
    {
      uint16_t l = strlen(g_registered_params[param_idx].name);
      if (l > 255)
        l = 255;
      *(g_tx_pkt_buf+5) = (uint8_t)l;
      strncpy((char *)(g_tx_pkt_buf+6),
              g_registered_params[param_idx].name, l);
      comms_send_packet(0x17, l+1);
    }
  }
  else if (pkt_type == 0x18) // get parameter value
  {
    uint16_t param_idx;
    param_idx = *((uint16_t *)(payload));
    if (param_idx >= g_num_registered_params)
      comms_send_packet(0x18, 0);
    else
    {
      if (g_registered_params[param_idx].name[0] == 'f')
        *((float *)(g_tx_pkt_buf+5)) =
          *(float *)(g_registered_params[param_idx].val);
      else
        *((uint32_t *)(g_tx_pkt_buf+5)) =
          *(uint32_t *)(g_registered_params[param_idx].val);
      comms_send_packet(0x18, 4);
    }
  }
  else if (pkt_type == 0x19) // set parameter value
  {
    uint16_t param_idx;
    param_idx = *((uint16_t *)(payload));
    if (param_idx >= g_num_registered_params)
      comms_send_packet(0x19, 1);
    else
    {
      if (g_registered_params[param_idx].name[0] == 'f')
        *(float *)(g_registered_params[param_idx].val) =
          *((float *)(payload+2));
      else
        *(uint32_t *)(g_registered_params[param_idx].val) =
          *((uint32_t *)(payload+2));
      comms_send_packet(0x19, 0);
    }
  }
  else if (pkt_type == 0x1a) // commit parameters to flash
  {
    params_save_all_to_flash();
    comms_send_packet(0x1a, 1);
   }
  else if (pkt_type == 0x1b) // reload parameter from flash
  {
    params_load_all_from_flash();
    comms_send_packet(0x1b, 0);
  }
  else if (pkt_type == 0x1c) // set parameters to default
  {
    params_set_all_default();
    comms_send_packet(0x1c, 0);
  }
  else if (pkt_type == 0xfd) // announce myself
  {
    // todo: figure out a way to add a random delay here, to avoid collisions
    *((uint16_t *)(g_tx_pkt_buf+ 5)) = g_comms_rs485_address;
    comms_send_packet(0xfd, 1);
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
      g_comms_rs485_address = new_addr;
      comms_send_packet(0xfe, 0);
    }
  }
#endif
}

