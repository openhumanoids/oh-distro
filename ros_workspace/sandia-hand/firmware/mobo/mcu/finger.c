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

#include "sandia_hand/hand_packets.h"
#include "finger.h"
#include <stdio.h>
#include "common_sam3x/sam3x.h"
#include "fpga.h"
#include "config.h"

// hardware connections:
// PA15 = RS485_SEL
// PA19 = MCU_RS485_MUX_A0
// PC7  = MCU_RS485_MUX_A1
// PB27 = MCU_RS485_MUX_A2
// PC28 = MCU_RS485_0_DE
// PD6  = MCU_RS485_1_DE
// PA0  = MCU_RS485_2_DE  
// PB14 = MCU_RS485_3_DE
// PD0  = MCU_RS485_4_DE
// PD4  = TXD3 = MCU_MUXED_DI
// PD5  = RXD3 = MCU_MUXED_RO

static void finger_broadcast_raw(const uint8_t *data, const uint16_t data_len);
static uint8_t g_finger_status_request = 0;
static volatile uint16_t g_finger_autopoll_timeout = 0;
static void finger_mobo_udp_rs485_enable(uint8_t enable);
#define FINGER_TX_QUEUE_LEN 512
static uint8_t  g_finger_tx_queue[5][FINGER_TX_QUEUE_LEN];
static uint32_t g_finger_tx_queue_len[5] = {0};
static uint32_t g_finger_systick_count = 0;
static void finger_flush_tx_queues();

typedef struct { Pio *pio; uint32_t pin_idx; } rs485_de_t;

/*
// this is for alpha rev right hand:
static const rs485_de_t g_finger_rs485_de[5] =
  { { PIOC, PIO_PC28 },
    { PIOD, PIO_PD6  },
    { PIOA, PIO_PA0  },
    { PIOB, PIO_PB14 },
    { PIOD, PIO_PD0  } };
*/
// todo: read magic byte in bootloader to indicate RH or LH
// this is for right hand
// RH map: 0->4   1->1   2->2   3->3   4->0

// this gets overwritten during init() depending on if it's RH or LH 
static rs485_de_t g_finger_rs485_de[5] =
  { { PIOD, PIO_PD0  },
    { PIOD, PIO_PD6  },
    { PIOA, PIO_PA0  },
    { PIOB, PIO_PB14 },
    { PIOC, PIO_PC28 } };

/*
// this is for left hand
// LH map: 0->1   1->0   2->3   3->2   4->4
static const rs485_de_t g_finger_rs485_de[5] =
  { 
    { PIOD, PIO_PD6  }, // mcu_rs485_1_de
    { PIOC, PIO_PC28 }, // mcu_rs485_0_de
    { PIOB, PIO_PB14 }, // mcu_rs485_3_de
    { PIOA, PIO_PA0  }, // mcu_rs485_2_de
    { PIOD, PIO_PD0  }  // mcu_rs485_4_de
  };
*/

void finger_init()
{
  if (config_is_left_hand())
  {
    g_finger_rs485_de[0].pio = PIOD;
    g_finger_rs485_de[0].pin_idx = PIO_PD6;
    g_finger_rs485_de[1].pio = PIOC;
    g_finger_rs485_de[1].pin_idx = PIO_PC28;
    g_finger_rs485_de[2].pio = PIOB;
    g_finger_rs485_de[2].pin_idx = PIO_PB14;
    g_finger_rs485_de[3].pio = PIOA;
    g_finger_rs485_de[3].pin_idx = PIO_PA0;
    g_finger_rs485_de[4].pio = PIOD;
    g_finger_rs485_de[4].pin_idx = PIO_PD0;
  }
  else
  {
    g_finger_rs485_de[0].pio = PIOD;
    g_finger_rs485_de[0].pin_idx = PIO_PD0;
    g_finger_rs485_de[1].pio = PIOD;
    g_finger_rs485_de[1].pin_idx = PIO_PD6;
    g_finger_rs485_de[2].pio = PIOA;
    g_finger_rs485_de[2].pin_idx = PIO_PA0;
    g_finger_rs485_de[3].pio = PIOB;
    g_finger_rs485_de[3].pin_idx = PIO_PB14;
    g_finger_rs485_de[4].pio = PIOC;
    g_finger_rs485_de[4].pin_idx = PIO_PC28;
  }

  PMC->PMC_PCER0 |= (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_PIOC) | 
                    (1 << ID_PIOD) | (1 << ID_USART3);
  const uint32_t pioa_pins = PIO_PA15 | PIO_PA19 | PIO_PA0;
  const uint32_t piob_pins = PIO_PB14 | PIO_PB27;
  const uint32_t pioc_pins = PIO_PC7 | PIO_PC28;
  const uint32_t piod_pins = PIO_PD6 | PIO_PD0; 
  PIOA->PIO_PER = PIOA->PIO_OER = PIOA->PIO_CODR = pioa_pins;
  PIOB->PIO_PER = PIOB->PIO_OER = PIOB->PIO_CODR = piob_pins;
  PIOC->PIO_PER = PIOC->PIO_OER = PIOC->PIO_CODR = pioc_pins;
  PIOD->PIO_PER = PIOD->PIO_OER = PIOD->PIO_CODR = piod_pins;
  PIOD->PIO_PDR = PIO_PD4 | PIO_PD5; // enable peripheral control of pd4, pd5
  PIOD->PIO_ABSR |= PIO_PD4B_TXD3 | PIO_PD5B_RXD3; // select peripheral B
  // assert PA15 (RS485_SEL) so the ARM maintains control of rs485 channels
  PIOA->PIO_SODR = PIO_PA15;
  // configure USART3 for 2 megabit
  USART3->US_CR = US_CR_RSTRX | US_CR_RSTTX |
                  US_CR_RXDIS | US_CR_TXDIS |
                  US_CR_RSTSTA;
  USART3->US_MR = US_MR_USART_MODE_NORMAL  | // regular UART, nothing fancy
                  US_MR_USCLKS_MCK         | // use master clock ( F_CPU )
                  US_MR_CHRL_8_BIT         | // 8 bit characters
                  US_MR_PAR_NO             | // no parity bit
                  US_MR_NBSTOP_1_BIT       ; // 1 stop bit
  USART3->US_IDR = 0xffffffff; // no interrupts
  USART3->US_BRGR = (F_CPU / 2000000) / 16; // make it a 2 megabit channel
  USART3->US_CR = US_CR_TXEN | US_CR_RXEN;
  fpga_spi_txrx(FPGA_SPI_REG_FINGER_BAUD_DIV | FPGA_SPI_WRITE, 50); // 2 Mb
}

static uint16_t finger_calc_crc(uint8_t *pkt)
{
  uint16_t crc = 0, pkt_len = *(uint16_t *)(pkt+2);
  uint8_t d, crc_highbit;
  for (uint32_t i = 0; i < (uint32_t)pkt_len+5; i++)
  {
    d = pkt[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc_highbit = (crc >> 8) & 0x80;
      crc <<= 1;
      if ((d & 0x80) ^ crc_highbit)
        crc ^= 0x1021; // CRC-16 CCITT polynomial
      d <<= 1;
    }
  }
  return crc;
}

void finger_set_control_mode(uint8_t finger_idx, uint8_t control_mode)
{
  if (finger_idx > 3)
    return;
  // craft finger packet and send it out
  uint8_t pkt[50];
  pkt[0] = 0x42;
  pkt[1] = 10; // generic finger address
  *((uint16_t *)(&pkt[2])) = 13; // 1 mode byte + three 32-bit floats 
  pkt[4] = 0x1d; // control mode packet id
  if (control_mode == FINGER_CONTROL_MODE_IDLE)
    pkt[5] = 0;
  else if (control_mode == FINGER_CONTROL_MODE_JOINT_POS)
    pkt[5] = 2;
  else
    return; // other modes are currently undefined
  for (int i = 6; i < 6+12; i++)
    pkt[i] = 0; // set target as (0,0,0)
  *((uint16_t *)(&pkt[18])) = finger_calc_crc(pkt);
  finger_enqueue_tx_raw(finger_idx, pkt, 20);
}

void finger_set_joint_pos(uint8_t finger_idx, float j0, float j1, float j2)
{
  if (finger_idx > 3)
    return;
  uint8_t pkt[50];
  pkt[0] = 0x42;
  pkt[1] = 10; // generic finger address
  *((uint16_t *)(&pkt[2])) = 13; // 1 mode byte + three 32-bit floats 
  pkt[4] = 0x1d; // control mode packet id
  pkt[5] = 2; // joint position mode
  *((float *)&pkt[6]) = j0;
  *((float *)&pkt[10]) = j1;
  *((float *)&pkt[14]) = j2;
  *((uint16_t *)(&pkt[18])) = finger_calc_crc(pkt);
  finger_enqueue_tx_raw(finger_idx, pkt, 20);
}

void finger_set_all_joint_angles_with_max_efforts(const float *joint_angles, 
                                                  const uint8_t *max_efforts,
                                                  const uint8_t control_mode)
{
  uint8_t pkt[50];
  for (int i = 0; i < 4; i++)
  {
    pkt[0] = 0x42;
    pkt[1] = 10;
    *((uint16_t *)(&pkt[2])) = 16; // 1 mode byte + three floats + 3 uint8
    pkt[4] = 0x1d; // control mode packet id
    pkt[5] = control_mode; // joint position mode with max efforts
    *((float *)&pkt[6])  = joint_angles[i*3  ];
    *((float *)&pkt[10]) = joint_angles[i*3+1];
    *((float *)&pkt[14]) = joint_angles[i*3+2];
    pkt[18] = max_efforts[i*3  ];
    pkt[19] = max_efforts[i*3+1];
    pkt[20] = max_efforts[i*3+2];
    *((uint16_t *)(&pkt[21])) = finger_calc_crc(pkt);
    finger_enqueue_tx_raw(i, pkt, 23);
  }
}

void finger_enqueue_tx_raw(uint8_t finger_idx,
                           const uint8_t *data, const uint16_t data_len)
{
  if (finger_idx > 4)
    return; // bogus
  // need to implement a proper circular buffer sometime. this will drop tx.
  __disable_irq();
  for (int i = 0; i < data_len; i++)
    g_finger_tx_queue[finger_idx][i] = data[i];
  g_finger_tx_queue_len[finger_idx] = data_len;
  __enable_irq();
}
 
void finger_tx_raw(const uint8_t finger_idx, 
                   const uint8_t *data, const uint16_t data_len)
{
  if (finger_idx > 4) // palm is the "fourth finger" since protocol is same
    return;
  /*
  printf("finger tx raw %d bytes:\r\n", data_len);
  for (int i = 0; i < data_len; i++)
    printf("  %d: 0x%02x\r\n", i, data[i]);
  */
  finger_mobo_udp_rs485_enable(0); // disable UDP relay of rs485
  // assert RS485_SEL so that the ARM has control of the rs485 transceivers
  PIOA->PIO_SODR = PIO_PA15;
  // typedef struct { Pio *pio; uint32_t pin_idx; } rs485_de_t;
  // drive the rs485 channel
  // TEMPORARY HACK: blast out on all rs485 channels
  /*
  for (int i = 0; i < 5; i++)
  {
    const rs485_de_t *de = &g_finger_rs485_de[i];
    de->pio->PIO_SODR = de->pin_idx;
  }
  */
  /*
  if (finger_idx == FINGER_THUMB) 
    USART3->US_MR |= US_MR_INVDATA; // thumb needs to be inverted, whoops
  else
    USART3->US_MR &= ~US_MR_INVDATA;
  */
  const rs485_de_t *de = &g_finger_rs485_de[finger_idx];
  de->pio->PIO_SODR = de->pin_idx;
  for (volatile int i = 0; i < 10; i++) { } // let driver ramp up
  while ((USART3->US_CSR & US_CSR_TXRDY) == 0) { }
  for (uint32_t i = 0; i < data_len; i++)
  {
    USART3->US_THR = data[i];
    while ((USART3->US_CSR & US_CSR_TXRDY) == 0) { }
  }
  while ((USART3->US_CSR & US_CSR_TXEMPTY) == 0) { } // wait to finish tx'ing
  for (volatile int i = 0; i < 10; i++) { } // wait a bit (why?)
  // release the rs485 channel
  de->pio->PIO_CODR = de->pin_idx;
  for (volatile int i = 0; i < 10; i++) { } // wait for rise time
  finger_mobo_udp_rs485_enable(1); // re-enable UDP relay of rs485
  // TEMPORARY HACK: blast out on all rs485 channels
  /*
  for (int i = 0; i < 5; i++)
  {
    const rs485_de_t *de = &g_finger_rs485_de[i];
    //de->pio->PIO_SODR = de->pin_idx;
    de->pio->PIO_CODR = de->pin_idx;
  }
  */
  // for now, let's allow the ARM to keep control of rs485.
}

void finger_broadcast_raw(const uint8_t *data, const uint16_t data_len)
{
  finger_mobo_udp_rs485_enable(0);
  PIOA->PIO_SODR = PIO_PA15; // assert ARM control of rs485 channels
  for (int i = 0; i < 5; i++)
  {
    const rs485_de_t *de = &g_finger_rs485_de[i];
    de->pio->PIO_SODR = de->pin_idx;
  }
  for (volatile int i = 0; i < 10; i++) { } // let driver ramp up
  while ((USART3->US_CSR & US_CSR_TXRDY) == 0) { }
  for (uint32_t i = 0; i < data_len; i++)
  {
    USART3->US_THR = data[i];
    while ((USART3->US_CSR & US_CSR_TXRDY) == 0) { }
  }
  while ((USART3->US_CSR & US_CSR_TXEMPTY) == 0) { } // wait to finish tx'ing
  for (volatile int i = 0; i < 10; i++) { } // wait a bit (why?)
  for (int i = 0; i < 5; i++)
  {
    const rs485_de_t *de = &g_finger_rs485_de[i];
    de->pio->PIO_CODR = de->pin_idx;
  }
  for (volatile int i = 0; i < 10; i++) { } // wait for rise time
  finger_mobo_udp_rs485_enable(1); // re-enable rs485 relay
}

void finger_idle()
{
  if (g_finger_status_request)
  {
    g_finger_status_request = 0;
    //printf("fsr\r\n");
    uint8_t pkt[50];
    pkt[0] = 0x42;
    pkt[1] = 10; // generic finger address
    *((uint16_t *)(&pkt[2])) = 0; // no payload
    pkt[4] = 0x21; // status request
    *((uint16_t *)(&pkt[5])) = finger_calc_crc(pkt);
    finger_broadcast_raw(pkt, 7);
  }
  const uint32_t t = g_finger_systick_count % g_finger_autopoll_timeout;
  if (!g_finger_autopoll_timeout || t > 2)
    finger_flush_tx_queues();
}

void finger_flush_tx_queues()
{
  volatile uint8_t  v_finger_tx_pkt_buf[FINGER_TX_QUEUE_LEN];
  volatile uint32_t v_finger_tx_pkt_len = 0;
  // flush any queued tx requests
  for (int i = 0; i < 5; i++)
  {
    // copy out tx buffer while not being interrupted by udp rx interrupts
    __disable_irq();
    v_finger_tx_pkt_len = g_finger_tx_queue_len[i];
    for (int j = 0; j < v_finger_tx_pkt_len; j++)
      v_finger_tx_pkt_buf[j] = g_finger_tx_queue[i][j];
    g_finger_tx_queue_len[i] = 0;
    __enable_irq();
    // fine if we get interrupted from here on
    if (v_finger_tx_pkt_len)
      finger_tx_raw(i, (uint8_t *)v_finger_tx_pkt_buf, 
                    (uint32_t)v_finger_tx_pkt_len);
  }
}

void finger_systick()
{
  g_finger_systick_count++;
  const uint32_t t = g_finger_systick_count % g_finger_autopoll_timeout;
  if (g_finger_autopoll_timeout && t == 0)
    g_finger_status_request = 1; // schedule query message to fingers
}

void finger_set_autopoll_rate(uint16_t hz)
{
  if (hz)
    g_finger_autopoll_timeout = 1000 / hz;
  else
    g_finger_autopoll_timeout = 0;
  //printf("fsar %d fat = %d\r\n", hz, g_finger_autopoll_timeout);
}

void finger_mobo_udp_rs485_enable(uint8_t enable)
{
  //printf("rs485 enable(%d)\r\n", enable);
  fpga_spi_txrx(FPGA_SPI_REG_RS485_UDP_TX | FPGA_SPI_WRITE, 
                enable ? 0x1f : 0);
}

void finger_set_all_effort_limits(const uint8_t mobo_max_effort)
{
  // todo: combine with single-finger function 
  uint8_t pkt[50];
  pkt[0] = 0x42;
  pkt[1] = 10; // generic finger address
  *((uint16_t *)(&pkt[2])) = 1; // no payload
  pkt[4] = 0x23; // set mobo effort limit
  pkt[5] = mobo_max_effort;
  *((uint16_t *)(&pkt[6])) = finger_calc_crc(pkt);
  finger_broadcast_raw(pkt, 8);
}

void finger_set_mobo_effort_limit(const uint8_t finger_idx, 
                                  const uint8_t mobo_max_effort)
{
  uint8_t pkt[50];
  pkt[0] = 0x42;
  pkt[1] = 10; // generic finger address
  *((uint16_t *)(&pkt[2])) = 1; // no payload
  pkt[4] = 0x23; // set mobo effort limit
  pkt[5] = mobo_max_effort;
  *((uint16_t *)(&pkt[6])) = finger_calc_crc(pkt);
  finger_enqueue_tx_raw(finger_idx, pkt, 8);
}

