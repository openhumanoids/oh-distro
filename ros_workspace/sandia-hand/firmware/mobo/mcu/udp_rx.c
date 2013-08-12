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
#include "power.h"
#include "finger.h"
#include "cam.h"
#include "fpga.h"
#include "flash.h"
#include "control.h"
#include "config.h"

void enet_udp_rx(uint8_t *pkt, const uint32_t len)
{
  udp_header_t *udp = (udp_header_t *)pkt;
  const uint16_t port = ntohs(udp->udp_dest_port);
  if (port != UDP_RX_PORT)
  {
    //printf("enet_udp_rx, unknown port %d\r\n", port);
    return;
  }
  uint8_t *udp_payload = pkt + sizeof(udp_header_t);
  uint32_t udp_payload_len = len - sizeof(udp_header_t);
  uint32_t cmd = *((uint32_t *)udp_payload);
  uint8_t *cmd_data = (uint8_t *)(udp_payload+4);
  // todo: this if..else if block has become way too large. factor it out
  // somehow into something more clean and efficient 
  if (cmd == CMD_ID_SET_FINGER_POWER_STATE)
  {
    set_finger_power_state_t *sfp = (set_finger_power_state_t *)cmd_data;
    if (sfp->finger_idx > 3)
      return; 
    if (sfp->finger_power_state > (uint8_t)POWER_ON)
      return;
    power_set(sfp->finger_idx, (power_state_t)sfp->finger_power_state);
  }
  else if (cmd == CMD_ID_SET_ALL_FINGER_POWER_STATES)
  {
    for (int i = 0; i < 4; i++)
      power_set(i, 
         (power_state_t)((set_all_finger_power_states_t *)cmd_data)->fps[i]);
  }
  else if (cmd == CMD_ID_SET_FINGER_CONTROL_MODE)
  {
    set_finger_control_mode_t *p = (set_finger_control_mode_t *)cmd_data;
    if (p->finger_idx > 3 || 
        p->finger_control_mode > FINGER_CONTROL_MODE_JOINT_POS)
      return;
    finger_set_control_mode(p->finger_idx, p->finger_control_mode);
  }
  else if (cmd == CMD_ID_SET_FINGER_JOINT_POS)
  {
    set_finger_joint_pos_t *p = (set_finger_joint_pos_t *)cmd_data;
    if (p->finger_idx > 3)
      return;
    finger_set_joint_pos(p->finger_idx, p->joint_0_radians, 
                         p->joint_1_radians, p->joint_2_radians);
  }
  else if (cmd == CMD_ID_CONFIGURE_CAMERA_STREAM)
  {
    configure_camera_stream_t *p = (configure_camera_stream_t *)cmd_data;
    cam_set_streams(p->cam_0_stream, p->cam_1_stream);
  }
  else if (cmd == CMD_ID_FINGER_RAW_TX)
  {
    finger_raw_tx_t *p = (finger_raw_tx_t *)cmd_data;
    if (p->finger_idx > 4 || p->tx_data_len > FINGER_RAW_TX_MAX_LEN)
      return;
    finger_enqueue_tx_raw(p->finger_idx, p->tx_data, p->tx_data_len);
  }
  else if (cmd == CMD_ID_SET_MOBO_STATUS_RATE)
    power_set_mobo_status_rate(
                      ((set_mobo_state_rate_t *)cmd_data)->mobo_state_hz);
  else if (cmd == CMD_ID_SET_FINGER_AUTOPOLL)
    finger_set_autopoll_rate(
                   ((set_finger_autopoll_t *)cmd_data)->finger_autopoll_hz);
  else if (cmd == CMD_ID_ENABLE_LOWVOLT_REGULATOR)
    power_enable_lowvolt_regulator(
                    ((enable_lowvolt_regulator_t *)cmd_data)->enable ? 1 : 0);
  else if (cmd == CMD_ID_READ_FPGA_FLASH_PAGE)
  {
    fpga_flash_page_t page;
    page.page_num = ((read_fpga_flash_page_t *)cmd_data)->page_num;
    page.page_status = FPGA_FLASH_PAGE_STATUS_READ;
    flash_read_page(page.page_num, page.page_data);
    enet_tx_packet(CMD_ID_FPGA_FLASH_PAGE, (uint8_t *)&page, sizeof(page));
  }
  else if (cmd == CMD_ID_FPGA_FLASH_PAGE)
  {
    fpga_flash_page_t *page = (fpga_flash_page_t *)cmd_data;
    if (page->page_status == FPGA_FLASH_PAGE_STATUS_WRITE_REQ)
    {
      flash_write_page(page->page_num, page->page_data);
      page->page_status = FPGA_FLASH_PAGE_STATUS_WRITE_ACK;
      enet_tx_packet(CMD_ID_FPGA_FLASH_PAGE, (uint8_t *)page, sizeof(*page));
    }
  }
  else if (cmd == CMD_ID_FPGA_FLASH_ERASE_SECTOR)
  {
    fpga_flash_erase_sector_t *req = (fpga_flash_erase_sector_t *)cmd_data;
    flash_erase_sector(req->sector_page_num);
    fpga_flash_erase_sector_ack_t res;
    res.sector_page_num = req->sector_page_num;
    enet_tx_packet(CMD_ID_FPGA_FLASH_ERASE_SECTOR_ACK, 
                   (uint8_t *)&res, sizeof(res));
  }
  else if (cmd == CMD_ID_MOBO_BOOT_CTRL)
  {
    mobo_boot_ctrl_t *p = (mobo_boot_ctrl_t *)cmd_data;
    if (p->boot_cmd == MOBO_BOOT_CTRL_RESET_REQUEST)
    {
      p->boot_cmd = MOBO_BOOT_CTRL_RESET_RESPONSE;
      enet_tx_packet(CMD_ID_MOBO_BOOT_CTRL, (uint8_t *)p, sizeof(*p));
      printf("rebooting...\r\n"); // this will take long enough for pkt to TX
      RSTC->RSTC_CR = RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;
    }
  }
  else if (cmd == CMD_ID_MOBO_PING)
  {
    mobo_ping_t *p = (mobo_ping_t *)cmd_data;
    if (p->state == MOBO_PING_REQUEST)
    {
      p->state = MOBO_PING_RESPONSE;
      enet_tx_packet(CMD_ID_MOBO_PING, (uint8_t *)p, sizeof(*p));
    }
  }
  else if (cmd == CMD_ID_HAND_JOINT_COMMANDS)
  {
    hand_joint_commands_t *p = (hand_joint_commands_t *)cmd_data;
    memcpy(control_target_joint_angles, p->joint_angles, sizeof(float)*12);
    memcpy(control_target_max_efforts, p->max_efforts, sizeof(uint8_t)*12);
    finger_set_all_joint_angles_with_max_efforts(control_target_joint_angles,
                                                 control_target_max_efforts, 4);
  }
  else if (cmd == CMD_ID_HAND_RELATIVE_JOINT_COMMANDS)
  {
    relative_joint_commands_t *p = (relative_joint_commands_t *)cmd_data;
    memcpy(control_target_joint_angles, 
           p->relative_joint_angles, sizeof(float)*12);
    memcpy(control_target_max_efforts, p->max_efforts, sizeof(uint8_t)*12);
    finger_set_all_joint_angles_with_max_efforts(control_target_joint_angles,
                                                 control_target_max_efforts, 5);
  }
  else if (cmd == CMD_ID_MOBO_SET_CURRENT_LIMIT)
  {
    set_mobo_current_limit_t *p = (set_mobo_current_limit_t *)cmd_data;
    if (p->pkt_state == MOBO_CURRENT_LIMIT_STATE_REQUEST)
    {
      power_set_mobo_current_limit(p->current_limit);
      p->pkt_state = MOBO_CURRENT_LIMIT_STATE_RESPONSE;
      enet_tx_packet(CMD_ID_MOBO_SET_CURRENT_LIMIT, (uint8_t *)p, sizeof(*p));
    }
  }
  else if (cmd == CMD_ID_MOBO_GET_HW_VERSION)
  {
    get_hw_version_t *p = (get_hw_version_t *)cmd_data;
    if (p->pkt_state == MOBO_GET_HW_VERSION_REQUEST)
    {
      p->version = config_get_hw_version();
      p->pkt_state = MOBO_GET_HW_VERSION_RESPONSE;
      enet_tx_packet(CMD_ID_MOBO_GET_HW_VERSION, (uint8_t *)p, sizeof(*p));
    }
  }
  else if (cmd == CMD_ID_MOBO_SET_DEST_PORT)
  {
    set_dest_port_t *p = (set_dest_port_t *)cmd_data;
    if (p->pkt_state == MOBO_SET_DEST_PORT_REQUEST)
    {
      enet_set_udp_base_port(p->port);
      p->pkt_state = MOBO_SET_DEST_PORT_RESPONSE;
      enet_tx_packet(CMD_ID_MOBO_SET_DEST_PORT, (uint8_t *)p, sizeof(*p));
    }
  }
  else
    printf("  unhandled cmd %d\r\n", cmd);
}

