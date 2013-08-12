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
#include "fpga.h"
#include "flash.h"
#include "config.h"

// hardware connections:
//   PB0 = EREFCK
//   PB1 = ETX_EN
//   PB2 = ETX_0
//   PB3 = ETX_1
//   PB4 = ERX_DV
//   PB5 = ERX_0
//   PB6 = ERX_1
//   PB7 = ERX_ER
static uint8_t g_enet_arp_valid = 0;

// structure definitions taken from ASF 3.5.1,  /sam/drivers/emac/emac.h
typedef struct emac_rx_descriptor {
  union emac_rx_addr {
    uint32_t val;
    struct emac_rx_addr_bm {
      uint32_t b_ownership:1, // user clear, EMAC sets this to 1 after rx
      b_wrap:1,   // marks last descriptor in receive buffer 
      addr_dw:30; // address in number of DW
    } bm;
  } addr; // address, wrap & ownership 
  union emac_rx_status {
    uint32_t val;
    struct emac_rx_status_bm {
      uint32_t len:12,       // Length of frame including FCS 
      offset:2,              // rx buf offset, 13:12 of frame len (jumbo frame)
      b_sof:1,               // Start of frame 
      b_eof:1,               // End of frame 
      b_cfi:1,               // Concatenation Format Indicator 
      vlan_priority:3,       // VLAN priority (if VLAN detected) 
      b_priority_detected:1, // Priority tag detected 
      b_vlan_detected:1,     // VLAN tag detected 
      b_type_id_match:1,     // Type ID match
      b_addr4match:1,        // Address register 4 match
      b_addr3match:1,        // Address register 3 match
      b_addr2match:1,        // Address register 2 match
      b_addr1match:1,        // Address register 1 match
      reserved:1,
      b_ext_addr_match:1,    // External address match
      b_uni_hash_match:1,    // Unicast hash match
      b_multi_hash_match:1,  // Multicast hash match
      b_boardcast_detect:1;  // Global broadcast address detected
    } bm;
  } status;
} __attribute__ ((packed, aligned(8))) emac_rx_descriptor_t; 

typedef struct emac_tx_descriptor {
  uint32_t addr;
  union emac_tx_status {
    uint32_t val;
    struct emac_tx_status_bm {
      uint32_t len:11, // length of frame
      reserved:4,
      b_last_buffer:1, // is last buffer in frame?
      b_no_crc:1,      // no crc
      reserved1:10,
      b_exhausted:1,   // buffer exhausted mid frame
      b_underrun:1,    // tx underrun
      b_error:1,       // retry fail... error
      b_wrap:1,        // ring buffer wraparound bit
      b_used:1;        // user clear, EMAC sets this to 1 after tx
    } bm;
  } status;
} __attribute__ ((packed, aligned(8))) emac_tx_descriptor_t;

#define ENET_MAX_PKT_SIZE 1550

/////////////////////////////////////////////////////////////////////////////
typedef struct
{
  eth_header_t eth;
  uint16_t arp_hw_type         : 16;
  uint16_t arp_proto_type      : 16;
  uint8_t  arp_hw_addr_len     :  8;
  uint8_t  arp_proto_addr_len  :  8;
  uint16_t arp_operation       : 16;
  uint8_t  arp_sender_hw_addr[6];
  uint32_t arp_sender_proto_addr;
  uint8_t  arp_target_hw_addr[6];
  uint32_t arp_target_proto_addr;
} __attribute__((packed)) arp_pkt_t;
static const uint16_t ARP_ETHERTYPE = 0x0806;
static const uint16_t ARP_HW_ETHERNET = 1;
static const uint16_t ARP_PROTO_IPV4 = 0x0800;
static const uint16_t ARP_OP_REQUEST = 1;
static const uint16_t ARP_OP_RESPONSE = 2;

/////////////////////////////////////////////////////////////////////////////
#define ICMP_MAX_DATA 200
typedef struct
{
  ip_header_t ip;
  uint8_t  icmp_type;
  uint8_t  icmp_code;
  uint16_t icmp_checksum;
  uint16_t icmp_id;
  uint16_t icmp_sequence;
} __attribute__((packed)) icmp_header_t;
static const uint8_t ICMP_ECHO_REPLY   = 0x00;
static const uint8_t ICMP_ECHO_REQUEST = 0x08;

/////////////////////////////////////////////////////////////////////////////
// globals

#define ENET_RX_BUFFERS 16
#define ENET_RX_UNITSIZE 128
volatile static emac_rx_descriptor_t __attribute__((aligned(8)))
                   g_enet_rx_desc[ENET_RX_BUFFERS];
volatile static uint8_t __attribute__((aligned(8))) 
                   g_enet_rx_buf[ENET_RX_BUFFERS * ENET_RX_UNITSIZE];
static uint8_t __attribute__((aligned(8)))
                   g_enet_rx_full_packet[ENET_MAX_PKT_SIZE];
static uint16_t g_enet_udp_base_port = 12321; // start here, move as requested

// keep the TX path simple for now. single big buffer.
#define ENET_TX_BUFFERS 1
#define ENET_TX_UNITSIZE ENET_MAX_PKT_SIZE
volatile static emac_tx_descriptor_t __attribute__((aligned(8)))
                   g_enet_tx_desc[ENET_TX_BUFFERS];
volatile static uint8_t __attribute__((aligned(8)))
                   g_enet_tx_buf[ENET_TX_BUFFERS * ENET_TX_UNITSIZE];
volatile static uint8_t __attribute__((aligned(8)))
                   g_enet_udp_tx_buf[ENET_MAX_PKT_SIZE];

// these values will get overwritten with whatever is in bootloader flash
static uint8_t g_enet_hand_mac[6] = {0xa4,0xf3,0xc1,0,0,0};
static uint32_t g_enet_hand_ip = 0x0a0a0102; 
static uint32_t g_enet_master_ip = 0x0a0a0101; 
static uint8_t g_enet_master_mac[6] = {0,0,0,0,0,0}; // to request via ARP

void enet_init()
{
  printf("enet init\r\n");
  for (int i = 0; i < 6; i++)
    g_enet_hand_mac[i] = config_get_hand_mac()[i];
  g_enet_hand_ip = config_get_hand_ip();
  g_enet_master_ip = config_get_master_ip();
  fpga_spi_txrx(FPGA_SPI_REG_ETH_SOURCE_ADDR_0 | FPGA_SPI_WRITE,
                 (uint16_t)g_enet_hand_mac[5] |
                ((uint16_t)g_enet_hand_mac[4] << 8));
  fpga_spi_txrx(FPGA_SPI_REG_ETH_SOURCE_ADDR_1 | FPGA_SPI_WRITE,
                 (uint16_t)g_enet_hand_mac[3] |
                ((uint16_t)g_enet_hand_mac[2] << 8));
  fpga_spi_txrx(FPGA_SPI_REG_ETH_SOURCE_ADDR_2 | FPGA_SPI_WRITE,
                 (uint16_t)g_enet_hand_mac[1] |
                ((uint16_t)g_enet_hand_mac[0] << 8));

  fpga_spi_txrx(FPGA_SPI_REG_IP_SOURCE_ADDR_LO | FPGA_SPI_WRITE,
                 (uint16_t) g_enet_hand_ip);
  fpga_spi_txrx(FPGA_SPI_REG_IP_SOURCE_ADDR_HI | FPGA_SPI_WRITE,
                 (uint16_t)(g_enet_hand_ip >> 16));

  fpga_spi_txrx(FPGA_SPI_REG_IP_DEST_ADDR_LO | FPGA_SPI_WRITE,
                 (uint16_t) g_enet_master_ip);
  fpga_spi_txrx(FPGA_SPI_REG_IP_DEST_ADDR_HI | FPGA_SPI_WRITE,
                 (uint16_t)(g_enet_master_ip >> 16));

  PMC->PMC_PCER0 |= (1 << ID_PIOB);
  PMC->PMC_PCER1 |= (1 << (ID_EMAC - 32));
  PIOB->PIO_ABSR &= ~(PIO_PB0A_ETXCK | PIO_PB1A_ETXEN | 
                      PIO_PB2A_ETX0  | PIO_PB3A_ETX1  |
                      PIO_PB4A_ERXDV | PIO_PB7A_ERXER |
                      PIO_PB5A_ERX0  | PIO_PB6A_ERX1) ; // select peripheral A
  PIOB->PIO_PDR = PIO_PB0A_ETXCK | PIO_PB1A_ETXEN | 
                  PIO_PB2A_ETX0  | PIO_PB3A_ETX1  |
                  PIO_PB4A_ERXDV | PIO_PB7A_ERXER |
                  PIO_PB5A_ERX0  | PIO_PB6A_ERX1  ; // set peripheral control
  EMAC->EMAC_NCR = 0; // disable everything
  EMAC->EMAC_IDR = 0xffffffff; // disable all interrupts
  EMAC->EMAC_NCR |= EMAC_NCR_CLRSTAT; // reset statistics
  EMAC->EMAC_USRIO = EMAC_USRIO_RMII | EMAC_USRIO_CLKEN; // select RMII mode
  EMAC->EMAC_RSR = EMAC_RSR_OVR | EMAC_RSR_REC | EMAC_RSR_BNA; // clear rx flags
  EMAC->EMAC_TSR = EMAC_TSR_UBR | EMAC_TSR_COL | EMAC_TSR_RLES |
                   EMAC_TSR_BEX | EMAC_TSR_COMP | EMAC_TSR_UND; // and tx flags
  EMAC->EMAC_ISR; // drain interrupts
  EMAC->EMAC_NCFGR = EMAC_NCFGR_DRFCS     |  // drop FCS from rx packets
                     EMAC_NCFGR_PAE       |  // obey pause frames
                     EMAC_NCFGR_CAF       |  // promiscuous mode
                     EMAC_NCFGR_SPD       |  // 100 megabit
                     EMAC_NCFGR_FD        |  // full duplex
                     EMAC_NCFGR_CLK_MCK_64;  // mdio clock = mdc / 64

  for (int i = 0; i < ENET_RX_BUFFERS; i++)
  {
    g_enet_rx_desc[i].addr.val = (uint32_t)&g_enet_rx_buf[i * ENET_RX_UNITSIZE] 
                                 & 0xfffffffc; // make sure it's 8-byte aligned
    g_enet_rx_desc[i].status.val = 0; 
  }
  g_enet_rx_desc[ENET_RX_BUFFERS-1].addr.bm.b_wrap = 1; // end of ring buffer
  EMAC->EMAC_RBQP = (uint32_t)g_enet_rx_desc & 0xfffffffc;

  for (int i = 0; i < ENET_TX_BUFFERS; i++)
  {
    g_enet_tx_desc[i].addr = (uint32_t)g_enet_tx_buf;
    g_enet_tx_desc[i].status.val = 0; // clear all flags
    g_enet_tx_desc[i].status.bm.b_used = 1; // no need to send this guy
  }
  g_enet_tx_desc[ENET_TX_BUFFERS-1].status.bm.b_wrap = 1; // end of ring 
  EMAC->EMAC_TBQP = (uint32_t)g_enet_tx_desc;

  EMAC->EMAC_NCR |= EMAC_NCR_RE     | // enable receiver
                    EMAC_NCR_WESTAT | // enable stats
                    EMAC_NCR_TE;      // enable transmitter
  EMAC->EMAC_IER = EMAC_IER_RXUBR | // receive used bit read (overrun?)
                   EMAC_IER_ROVR  | // receive overrun
                   EMAC_IER_RCOMP ; // receive complete
  NVIC_SetPriority(EMAC_IRQn, 2); // lower priority than i2c
  NVIC_EnableIRQ(EMAC_IRQn);
}

static void enet_arp_rx(uint8_t *pkt, const uint32_t len)
{
  arp_pkt_t *arp_pkt = (arp_pkt_t *)pkt;
  if (ntohs(arp_pkt->arp_hw_type) != ARP_HW_ETHERNET || 
      ntohs(arp_pkt->arp_proto_type) != ARP_PROTO_IPV4)
  {
    printf("unknown ARP hw type (0x%x) or protocol type (0x%0x)\r\n",
           ntohs(arp_pkt->arp_hw_type), ntohs(arp_pkt->arp_proto_type));
    return; // this function only handles ARP for IPv4 over ethernet
  }
  uint16_t op = ntohs(arp_pkt->arp_operation);
  if (op == ARP_OP_REQUEST) 
  {
    int req_ip = ntohl(arp_pkt->arp_target_proto_addr);
    if (req_ip != g_enet_hand_ip)
    {
      //printf("ignoring ARP request for 0x%08x\r\n", req_ip);
      return; 
    }
    //printf("request for 0x%08x\r\n", req_ip);
    //const uint8_t *request_eth_addr = arp_pkt->sender_hw_addr;
    //const uint32_t *request_ip = htonl(arp_pkt->sender_proto_addr);
    arp_pkt_t response;
    for (int i = 0; i < 6; i++)
    {
      response.eth.eth_dest_addr[i] = arp_pkt->arp_sender_hw_addr[i];
      response.eth.eth_source_addr[i] = g_enet_hand_mac[i];
      response.arp_sender_hw_addr[i] = g_enet_hand_mac[i];
      response.arp_target_hw_addr[i] = arp_pkt->arp_sender_hw_addr[i];
    }
    response.eth.eth_ethertype = htons(ARP_ETHERTYPE);
    response.arp_sender_proto_addr = htonl(g_enet_hand_ip);
    response.arp_target_proto_addr = arp_pkt->arp_sender_proto_addr;
    response.arp_hw_type = htons(ARP_HW_ETHERNET);
    response.arp_proto_type = htons(ARP_PROTO_IPV4);
    response.arp_hw_addr_len = 6; // ethernet address length
    response.arp_proto_addr_len = 4; // IPv4 address length
    response.arp_operation = htons(ARP_OP_RESPONSE);
    enet_tx_raw((uint8_t *)&response, sizeof(response));
  }
  else if (op == ARP_OP_RESPONSE)
  {
    printf("arp response rx\r\n");
    if (arp_pkt->arp_sender_proto_addr == htonl(g_enet_master_ip))
    {
      for (int i = 0; i < 6; i++)
        g_enet_master_mac[i] = arp_pkt->arp_sender_hw_addr[i];
      printf("master MAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
             g_enet_master_mac[0], g_enet_master_mac[1],
             g_enet_master_mac[2], g_enet_master_mac[3],
             g_enet_master_mac[4], g_enet_master_mac[5]);
      // set master MAC register on fpga
      fpga_spi_txrx(FPGA_SPI_REG_ETH_DEST_ADDR_0 | FPGA_SPI_WRITE,
                     (uint16_t)g_enet_master_mac[5] |
                    ((uint16_t)g_enet_master_mac[4] << 8));
      fpga_spi_txrx(FPGA_SPI_REG_ETH_DEST_ADDR_1 | FPGA_SPI_WRITE,
                     (uint16_t)g_enet_master_mac[3] |
                    ((uint16_t)g_enet_master_mac[2] << 8));
      fpga_spi_txrx(FPGA_SPI_REG_ETH_DEST_ADDR_2 | FPGA_SPI_WRITE,
                     (uint16_t)g_enet_master_mac[1] |
                    ((uint16_t)g_enet_master_mac[0] << 8));
      g_enet_arp_valid = 1;
    }
  }
}

static void enet_request_master_mac()
{
  printf("arp.\r\n");
  arp_pkt_t request;
  for (int i = 0; i < 6; i++)
  {
    request.eth.eth_dest_addr[i] = 0xff; // broadcast it
    request.arp_target_hw_addr[i] = 0xff;
    request.eth.eth_source_addr[i] = g_enet_hand_mac[i];
    request.arp_sender_hw_addr[i] = g_enet_hand_mac[i];
  }
  request.arp_sender_proto_addr = htonl(g_enet_hand_ip);
  request.arp_target_proto_addr = htonl(g_enet_master_ip);
  request.eth.eth_ethertype = htons(ARP_ETHERTYPE);
  request.arp_hw_type = htons(ARP_HW_ETHERNET);
  request.arp_proto_type = htons(ARP_PROTO_IPV4);
  request.arp_hw_addr_len = 6; // ethernet address length
  request.arp_proto_addr_len = 4; // IPv4 address length
  request.arp_operation = htons(ARP_OP_REQUEST);
  enet_tx_raw((uint8_t *)&request, sizeof(request));
}

static void enet_add_ip_header_checksum(ip_header_t *ip)
{
  ip->ip_checksum = 0;
  uint32_t sum = 0;
  for (int word_idx = 0; word_idx < 10; word_idx++)
  {
    uint16_t word = *((uint16_t *)ip + sizeof(eth_header_t)/2 + word_idx);
    word = ntohs(word);
    sum += word;
    //printf("word %d: 0x%02x\r\n", word_idx, word);
  }
  sum += (sum >> 16);
  sum &= 0xffff;
  ip->ip_checksum = (uint16_t)htons(~sum);
  //printf("ip header checksum: 0x%04x\r\n", ip->ip_checksum);
}

static void enet_icmp_rx(uint8_t *pkt, const uint32_t len)
{
  //printf("enet_icmp_rx\r\n");
  icmp_header_t *icmp = (icmp_header_t *)pkt;
  if (icmp->icmp_type != ICMP_ECHO_REQUEST)
    return;
  static const int ICMP_RESPONSE_MAX_LEN = 300; // i have no idea
  uint8_t icmp_response_buf[ICMP_RESPONSE_MAX_LEN];
  uint16_t incoming_ip_len = ntohs(icmp->ip.ip_len);
  uint16_t icmp_data_len = incoming_ip_len - 20 - 8; // everything after icmp
  if (icmp_data_len > ICMP_RESPONSE_MAX_LEN - sizeof(icmp_header_t))
    icmp_data_len = ICMP_RESPONSE_MAX_LEN - sizeof(icmp_header_t);
  icmp_header_t *icmp_response = (icmp_header_t *)icmp_response_buf;
  for (int i = 0; i < 6; i++)
  {
    icmp_response->ip.eth.eth_dest_addr[i] = icmp->ip.eth.eth_source_addr[i];
    icmp_response->ip.eth.eth_source_addr[i] = g_enet_hand_mac[i];
  }
  icmp_response->ip.eth.eth_ethertype = htons(IP_ETHERTYPE);
  icmp_response->ip.ip_header_len = IP_HEADER_LEN;
  icmp_response->ip.ip_version = IP_VERSION;
  icmp_response->ip.ip_ecn = 0;
  icmp_response->ip.ip_diff_serv = 0;
  icmp_response->ip.ip_len = htons(incoming_ip_len);
  icmp_response->ip.ip_id = 0;
  icmp_response->ip.ip_flag_frag = htons(IP_DONT_FRAGMENT);
  icmp_response->ip.ip_ttl = icmp->ip.ip_ttl;
  icmp_response->ip.ip_proto = IP_PROTO_ICMP;
  icmp_response->ip.ip_checksum = 0;
  icmp_response->ip.ip_source_addr = htonl(g_enet_hand_ip);
  icmp_response->ip.ip_dest_addr = icmp->ip.ip_source_addr;
  icmp_response->icmp_type = ICMP_ECHO_REPLY;
  icmp_response->icmp_code = 0;
  icmp_response->icmp_checksum = 0;
  icmp_response->icmp_id = icmp->icmp_id;
  icmp_response->icmp_sequence = icmp->icmp_sequence;
  enet_add_ip_header_checksum(&icmp_response->ip);
  uint8_t *icmp_response_payload = icmp_response_buf + sizeof(icmp_header_t);
  uint8_t *icmp_request_payload = pkt + sizeof(icmp_header_t);
  for (int i = 0; i < icmp_data_len; i++)
    icmp_response_payload[i] = icmp_request_payload[i];
  uint32_t csum = 0;
  for (int word_idx = 0; word_idx < 4 + icmp_data_len / 2; word_idx++)
  {
    uint16_t word = *((uint16_t *)icmp_response + sizeof(ip_header_t)/2 + 
                      word_idx);
    word = ntohs(word);
    csum += word;
  }
  csum += (csum >> 16);
  csum &= 0xffff;
  icmp_response->icmp_checksum = htons(~csum);
  enet_tx_raw(icmp_response_buf, sizeof(icmp_header_t) + icmp_data_len);
}


static void enet_ip_rx(uint8_t *pkt, const uint32_t len)
{
  ip_header_t *ip = (ip_header_t *)pkt; 
  const uint8_t proto = ip->ip_proto;
  if (proto == IP_PROTO_ICMP)
    enet_icmp_rx(pkt, len);
  else if (proto == IP_PROTO_UDP)
    enet_udp_rx(pkt, len);
}

static void enet_packet_rx(uint8_t *pkt, const uint32_t len)
{
  eth_header_t *eth = (eth_header_t *)pkt;
  // check our mac address and broadcast. in future, perhaps check a range.
  int mac_match = 1, bcast_match = 1;
  for (int i = 0; i < 6; i++)
  {
    if (eth->eth_dest_addr[i] != g_enet_hand_mac[i])
      mac_match = 0;
    if (eth->eth_dest_addr[i] != 0xff)
      bcast_match = 0;
  }
  if (!mac_match && !bcast_match)
    return; // buh bye
  int ethertype = ntohs(eth->eth_ethertype);
  if (ethertype == IP_ETHERTYPE)
    enet_ip_rx(pkt, len);
  else if (ethertype == ARP_ETHERTYPE)
    enet_arp_rx(pkt, len);
  else
    printf("unknown ethertype: 0x%04x\r\n", ethertype);
}

void enet_vector()
{
  // read the flags to reset the interrupt 
  volatile uint32_t enet_isr = EMAC->EMAC_ISR;
  volatile uint32_t enet_rsr = EMAC->EMAC_RSR;
  volatile uint32_t enet_tsr = EMAC->EMAC_TSR;
  if ((enet_isr & EMAC_ISR_RCOMP) || (enet_rsr & EMAC_RSR_REC))
  {
    volatile uint32_t rsr_clear_flag = EMAC_RSR_REC;
    if (enet_rsr & EMAC_RSR_OVR)
      rsr_clear_flag |= EMAC_RSR_OVR;
    if (enet_rsr & EMAC_RSR_BNA)
      rsr_clear_flag |= EMAC_RSR_BNA;
    EMAC->EMAC_RSR = rsr_clear_flag;
    // spin through buffers and mark them as unowned
    // collect used buffers into single ethernet RX buffer
    static int s_rx_buf_idx = 0;
    static int s_rx_pkt_write_idx = 0;
    while (g_enet_rx_desc[s_rx_buf_idx].addr.bm.b_ownership)
    {
      volatile emac_rx_descriptor_t *desc = &g_enet_rx_desc[s_rx_buf_idx];
      uint8_t *buf = (uint8_t *)&g_enet_rx_buf[s_rx_buf_idx*ENET_RX_UNITSIZE];
      desc->addr.bm.b_ownership = 0; // clear buffer
      if (desc->status.bm.b_sof)
        s_rx_pkt_write_idx = 0; // ensure we reset this
      const int buf_data_len = !desc->status.bm.len ? ENET_RX_UNITSIZE :
                               (desc->status.bm.len - s_rx_pkt_write_idx);
      /*
      printf("%d owned, size %d, sof %d, eof %d\r\n", 
             s_rx_buf_idx, buf_len, desc->status.bm.b_sof, 
             desc->status.bm.b_eof);
      */
      if (buf_data_len > 0 && 
          s_rx_pkt_write_idx + buf_data_len < ENET_MAX_PKT_SIZE)
      {
        memcpy(&g_enet_rx_full_packet[s_rx_pkt_write_idx], buf, buf_data_len);
        s_rx_pkt_write_idx += buf_data_len;
      }
      else
      {
        printf("AAAHH enet rx buffer trashed\r\n");
        s_rx_pkt_write_idx = 0;
      }
      if (desc->status.bm.b_eof)
      {
        enet_packet_rx(g_enet_rx_full_packet, s_rx_pkt_write_idx);
        s_rx_pkt_write_idx = 0; // to be really^n sure this gets reset... 
      }
      s_rx_buf_idx = ++s_rx_buf_idx % ENET_RX_BUFFERS; // advance in ring
    }
  }
}

void enet_tx_raw(const uint8_t *pkt, uint16_t pkt_len)
{
  g_enet_tx_desc[0].status.bm.b_used = 0; // we're monkeying with it
  // for now, we just crush whatever is in the tx buffer.
  if (pkt_len > ENET_TX_UNITSIZE)
    pkt_len = ENET_TX_UNITSIZE; // save memory from being brutally crushed
  /*
  printf("enet_tx_raw %d bytes:\r\n", pkt_len)
  for (int i = 0; i < pkt_len; i++)
    printf("%d: 0x%02x\r\n", i, pkt[i]);
  */
  memcpy((uint8_t *)g_enet_tx_buf, pkt, pkt_len);
  g_enet_tx_desc[0].status.bm.b_last_buffer = 1;
  g_enet_tx_desc[0].status.bm.len = pkt_len;
  EMAC->EMAC_NCR |= EMAC_NCR_TSTART; // kick off TX DMA
}

static int enet_master_mac_valid()
{
  for (int i = 0; i < 6; i++)
    if (g_enet_master_mac[i])
      return 1;
  return 0; // all zeros = invalid master MAC
}

uint8_t enet_tx_avail()
{
  return (g_enet_tx_desc[0].status.bm.b_last_buffer != 0);
}

void enet_idle()
{

}

void enet_systick()
{
  static int s_enet_systick_count = 0;
  if (++s_enet_systick_count % 2000 == 0)
  {
    if (!enet_master_mac_valid())
      enet_request_master_mac();
    //printf("enet 1hz systick\r\n");
  }
}

void enet_tx_udp(const uint8_t *payload, const uint16_t payload_len)
{
  udp_header_t *udp = (udp_header_t *)g_enet_udp_tx_buf;
  for (int i = 0; i < 6; i++)
  {
    udp->ip.eth.eth_dest_addr[i]   = g_enet_master_mac[i];
    udp->ip.eth.eth_source_addr[i] = g_enet_hand_mac[i];
  }
  udp->ip.eth.eth_ethertype = htons(IP_ETHERTYPE);
  udp->ip.ip_header_len = IP_HEADER_LEN;
  udp->ip.ip_version = IP_VERSION;
  udp->ip.ip_ecn = 0;
  udp->ip.ip_diff_serv = 0;
  udp->ip.ip_len = htons(20 + 8 + payload_len);
  udp->ip.ip_id = 0;
  udp->ip.ip_flag_frag = htons(IP_DONT_FRAGMENT);
  udp->ip.ip_ttl = 64; // no idea
  udp->ip.ip_proto = IP_PROTO_UDP;
  udp->ip.ip_checksum = 0;
  udp->ip.ip_source_addr = htonl(g_enet_hand_ip);
  udp->ip.ip_dest_addr = htonl(g_enet_master_ip);
  udp->udp_source_port = htons(g_enet_udp_base_port);
  udp->udp_dest_port = htons(g_enet_udp_base_port);
  udp->udp_len = htons(8 + payload_len);
  udp->udp_checksum = 0; // IPv4 UDP checksum is optional. 
  enet_add_ip_header_checksum(&udp->ip);
  memcpy((uint8_t *)udp + sizeof(udp_header_t), payload, payload_len);
  enet_tx_raw((uint8_t *)udp, sizeof(udp_header_t) + payload_len);
}

// todo: when the dust settles, combine this functionality with prior function
void enet_tx_packet(const uint32_t packet_id, 
                    const uint8_t *packet, const uint16_t packet_len)
{
  udp_header_t *udp = (udp_header_t *)g_enet_udp_tx_buf;
  for (int i = 0; i < 6; i++)
  {
    udp->ip.eth.eth_dest_addr[i]   = g_enet_master_mac[i];
    udp->ip.eth.eth_source_addr[i] = g_enet_hand_mac[i];
  }
  udp->ip.eth.eth_ethertype = htons(IP_ETHERTYPE);
  udp->ip.ip_header_len = IP_HEADER_LEN;
  udp->ip.ip_version = IP_VERSION;
  udp->ip.ip_ecn = 0;
  udp->ip.ip_diff_serv = 0;
  udp->ip.ip_len = htons(20 + 8 + 4 + packet_len);
  udp->ip.ip_id = 0;
  udp->ip.ip_flag_frag = htons(IP_DONT_FRAGMENT);
  udp->ip.ip_ttl = 64; // no idea
  udp->ip.ip_proto = IP_PROTO_UDP;
  udp->ip.ip_checksum = 0;
  udp->ip.ip_source_addr = htonl(g_enet_hand_ip);
  udp->ip.ip_dest_addr = htonl(g_enet_master_ip);
  udp->udp_source_port = htons(g_enet_udp_base_port);
  udp->udp_dest_port = htons(g_enet_udp_base_port);
  udp->udp_len = htons(8 + 4 + packet_len);
  udp->udp_checksum = 0; // IPv4 UDP checksum is optional. 
  enet_add_ip_header_checksum(&udp->ip);
  *((uint32_t *)(((uint8_t *)udp) + sizeof(udp_header_t))) = packet_id;
  memcpy(((uint8_t *)udp) + sizeof(udp_header_t) + 4, packet, packet_len);
  enet_tx_raw((uint8_t *)udp, sizeof(udp_header_t) + 4 + packet_len);
}

uint8_t enet_arp_valid()
{
  return g_enet_arp_valid;
}

uint16_t enet_get_udp_base_port()
{
  return g_enet_udp_base_port;
}

void enet_set_udp_base_port(const uint16_t port)
{
  g_enet_udp_base_port = port;
  fpga_spi_txrx(FPGA_SPI_REG_UDP_DEST_PORT | FPGA_SPI_WRITE, port);
}

