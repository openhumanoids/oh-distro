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

#ifndef MOBO_MCU_ENET_H
#define MOBO_MCU_ENET_H

#include <stdint.h>

void enet_init();
void enet_vector();
void enet_tx_raw(const uint8_t *pkt, uint16_t pkt_len);
uint8_t enet_tx_avail();
void enet_idle(); // handle non time-critical ethernet tasks
void enet_systick();
void enet_tx_udp(const uint8_t *payload, const uint16_t payload_len);
void enet_tx_packet(const uint32_t packet_id, 
                    const uint8_t *packet, 
                    const uint16_t packet_len); // uses UDP
void enet_udp_rx(uint8_t *pkt, const uint32_t len); // defined by user app
uint8_t enet_arp_valid();

typedef struct 
{
  uint8_t  eth_dest_addr[6];
  uint8_t  eth_source_addr[6];
  uint16_t eth_ethertype : 16;
} __attribute__((packed)) eth_header_t;

/////////////////////////////////////////////////////////////////////////////
typedef struct 
{
  eth_header_t eth;
  uint8_t  ip_header_len   :  4;
  uint8_t  ip_version      :  4;
  uint8_t  ip_ecn          :  2;
  uint8_t  ip_diff_serv    :  6;
  uint16_t ip_len          : 16;
  uint16_t ip_id           : 16;
  uint16_t ip_flag_frag    : 16;
  uint8_t  ip_ttl          :  8;
  uint8_t  ip_proto        :  8;
  uint16_t ip_checksum     : 16;
  uint32_t ip_source_addr  : 32;
  uint32_t ip_dest_addr    : 32;
} __attribute__((packed)) ip_header_t;
static const uint16_t IP_ETHERTYPE = 0x0800;
static const uint8_t  IP_HEADER_LEN = 5;
static const uint8_t  IP_VERSION = 4;
static const uint8_t  IP_PROTO_ICMP = 0x01;
static const uint8_t  IP_PROTO_UDP = 0x11;
static const uint16_t IP_DONT_FRAGMENT = 0x4000;

/////////////////////////////////////////////////////////////////////////////
typedef struct
{
  ip_header_t ip;
  uint16_t udp_source_port;
  uint16_t udp_dest_port;
  uint16_t udp_len;
  uint16_t udp_checksum;
} __attribute__((packed)) udp_header_t;

uint16_t enet_get_udp_base_port();
void     enet_set_udp_base_port(const uint16_t port);
#define  UDP_RX_PORT 12321


#define htonl(x) (__REV((uint32_t)x))
#define htons(x) (__REV16((uint16_t)x))
#define ntohl(x) (htonl(x))
#define ntohs(x) (htons(x))



#endif

