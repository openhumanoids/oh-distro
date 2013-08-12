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

#include "console.h"
#include "sam3s/sam3s.h"

static uint32_t g_console_init_complete = 0;

void console_init()
{
  PMC->PMC_PCER0 |= (1 << ID_UART1) | (1 << ID_PIOB); // enable clock gates
  PIOB->PIO_OER = PIO_PB3A_UTXD1; // output enable
  PIOB->PIO_PDR = PIO_PB3A_UTXD1 | PIO_PB2A_URXD1; // enable peripheral control
  PIOB->PIO_ABCDSR[0] &= ~(PIO_PB2A_URXD1 | PIO_PB3A_UTXD1); // select periph
  PIOB->PIO_ABCDSR[1] &= ~(PIO_PB2A_URXD1 | PIO_PB3A_UTXD1); // select periph
  UART1->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX |
                   UART_CR_RXDIS | UART_CR_TXDIS |
                   UART_CR_RSTSTA;
  UART1->UART_MR = UART_MR_PAR_NO |
                   UART_MR_CHMODE_NORMAL;
  UART1->UART_IDR = 0xffffffff; // no interrupts plz
  UART1->UART_BRGR = (64000000 / 115200) / 16; // make a 1 MBaud serial console
  UART1->UART_CR = UART_CR_TXEN | UART_CR_RXEN;
  g_console_init_complete = 1;
}

void console_send_block(uint8_t *block, uint32_t len)
{
  // if it ever matters, make this interrupt-driven
  if (!g_console_init_complete)
    return;
  while ((UART1->UART_SR & UART_SR_TXRDY) == 0) { }
  for (uint32_t i = 0; i < len; i++)
  {
    UART1->UART_THR = block[i];
    while ((UART1->UART_SR & UART_SR_TXRDY) == 0) { }
  }
  while (!(UART1->UART_SR & UART_SR_TXEMPTY)) { } // spin until byte is done
}

