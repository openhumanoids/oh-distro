/*  Software License Agreement (Apache License)
 *
 *  Copyright 2013 Open Source Robotics Foundation
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

#include "enet.h"
#include "common_sam3x/sam3x.h"
#include <stdio.h>
#include <string.h>
#include "sandia_hand/hand_packets.h"
#include "fpga.h"
#include "flash.h"
#include "boot.h"

// hardware connections:
//   PB0 = EREFCK
//   PB1 = ETX_EN
//   PB2 = ETX_0
//   PB3 = ETX_1
//   PB4 = ERX_DV
//   PB5 = ERX_0
//   PB6 = ERX_1
//   PB7 = ERX_ER

#define MOBO_BL_UDP_HAND_PORT 12321

void udp_rx_write_flash_page(const uint32_t page_num, uint8_t *page_data)
{
  //printf("writing page %d...\r\n", page_num);
  if (page_num < 128 || page_num > 2048)
    return; // bogus page number; either within bootloader or non-existant.
  EFC0->EEFC_FMR = EEFC_FMR_FWS(6); // per errata in datasheet
  EFC0->EEFC_FMR = EEFC_FMR_FWS(6); // ditto
  volatile uint32_t bank_id = (page_num < 1024 ? 0 : 1);
  volatile Efc *efc = (bank_id == 0 ? EFC0 : EFC1);
  volatile uint32_t *read_addr = (uint32_t *)page_data;
  volatile uint32_t *write_addr = (uint32_t *)(0x80000 + 256 * page_num);
  for (volatile uint32_t write_count = 0; write_count < 64; write_count++)
    *write_addr++ = *read_addr++;
  while ((efc->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // spin...
  // gratuitous hack to jump to the IAP function in ROM to write a flash page
  typedef uint32_t (*iap_fp)(uint32_t, uint32_t);
  iap_fp iap = *((iap_fp *)(IROM_ADDR + 8)); // magic IAP pointer in ROM.
  const uint32_t bank_page_num = (bank_id == 0 ? page_num : page_num - 1024);
  iap(bank_id, EEFC_FCR_FKEY(0x5A) | EEFC_FCR_FARG(bank_page_num) |
      EEFC_FCR_FCMD(0x03));
  while ((efc->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // spin...
  EFC0->EEFC_FMR = EEFC_FMR_FWS(4); // reset it so we can run faster now
  EFC1->EEFC_FMR = EEFC_FMR_FWS(4); // reset it so we can run faster now
}

void enet_udp_rx(uint8_t *pkt, const uint32_t len)
{
  udp_header_t *udp = (udp_header_t *)pkt;
  const uint16_t port = ntohs(udp->udp_dest_port);
  if (port != MOBO_BL_UDP_HAND_PORT)
  {
    //printf("enet_udp_rx, unknown port %d\r\n", port);
    return;
  }
  //printf("bootloader udp rx %d bytes\r\n", len);
  const uint8_t *udp_payload = pkt + sizeof(udp_header_t);
  const uint32_t udp_payload_len = len - sizeof(udp_header_t);
  const uint32_t cmd = *((uint32_t *)udp_payload);
  uint8_t *cmd_data = (uint8_t *)(udp_payload+4);
  if (cmd == CMD_ID_BL_MOBO_MCU_FLASH_PAGE)
  {
    mobo_mcu_flash_page_t *p = (mobo_mcu_flash_page_t *)cmd_data;
    if (p->page_status == MOBO_MCU_FLASH_PAGE_STATUS_READ_REQ)
    {
      //printf("flash page read request: page %d\r\n", p->page_num);
      if (p->page_num >= 2048)
        return; // page doesn't exist on this chip. ignore request.
      p->page_status = MOBO_MCU_FLASH_PAGE_STATUS_READ_RES; // response 
      for (int offset_word = 0; offset_word < 64; offset_word++)
        *((uint32_t *)(p->page_data + 4 * offset_word)) =
            *((uint32_t *)(0x80000 + p->page_num * 256 + 4 * offset_word));
      enet_tx_packet(CMD_ID_BL_MOBO_MCU_FLASH_PAGE, (uint8_t *)p, sizeof(*p));
    }
    else if (p->page_status == MOBO_MCU_FLASH_PAGE_STATUS_WRITE_REQ)
    {
      if (p->page_num < 128 || p->page_num > 2048)
        return; // page is either within bootloader, or non existent. ignore.
      p->page_status = MOBO_MCU_FLASH_PAGE_STATUS_WRITE_RES;
      udp_rx_write_flash_page(p->page_num, p->page_data);
      enet_tx_packet(CMD_ID_BL_MOBO_MCU_FLASH_PAGE, (uint8_t *)p, sizeof(*p));
    }
    else
      printf("unexpected mcu flash page status: %d\r\n", p->page_status);
  }
  else if (cmd == CMD_ID_MOBO_BOOT_CTRL)
  {
    mobo_boot_ctrl_t *p = (mobo_boot_ctrl_t *)cmd_data;
    if (p->boot_cmd == MOBO_BOOT_CTRL_BL_AUTOBOOT_HALT_REQUEST)
    {
      p->boot_cmd = MOBO_BOOT_CTRL_BL_AUTOBOOT_HALT_RESPONSE;
      boot_enabled = 0;
      enet_tx_packet(CMD_ID_MOBO_BOOT_CTRL, (uint8_t *)p, sizeof(*p));
    }
    else if (p->boot_cmd == MOBO_BOOT_CTRL_BL_BOOT_REQUEST)
    {
      p->boot_cmd = MOBO_BOOT_CTRL_BL_BOOT_RESPONSE;
      enet_tx_packet(CMD_ID_MOBO_BOOT_CTRL, (uint8_t *)p, sizeof(*p));
      printf("booting...\r\n");
      boot_requested = 1; // we'll then bail from the idle() func
    }
  }
  else
    printf("unhandled cmd: %d\r\n", cmd);
  // todo: add reset command to the application code, so we don't
  // have to power-cycle to get back to bootloader mode.
}

