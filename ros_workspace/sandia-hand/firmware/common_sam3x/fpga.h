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

#ifndef FPGA_H
#define FPGA_H

#include <stdint.h>

#define FPGA_SPI_REG_MASTER_CFG          0
#define FPGA_SPI_REG_MDIO_CFG            1
#define FPGA_SPI_REG_MDIO_WDATA          2
#define FPGA_SPI_REG_CAM_CFG             3
#define FPGA_SPI_REG_UDP_SOURCE_PORT     4
#define FPGA_SPI_REG_UDP_DEST_PORT       5
#define FPGA_SPI_REG_IP_SOURCE_ADDR_LO   6
#define FPGA_SPI_REG_IP_SOURCE_ADDR_HI   7
#define FPGA_SPI_REG_IP_DEST_ADDR_LO     8
#define FPGA_SPI_REG_IP_DEST_ADDR_HI     9
#define FPGA_SPI_REG_ETH_SOURCE_ADDR_0  10
#define FPGA_SPI_REG_ETH_SOURCE_ADDR_1  11
#define FPGA_SPI_REG_ETH_SOURCE_ADDR_2  12
#define FPGA_SPI_REG_ETH_DEST_ADDR_0    13
#define FPGA_SPI_REG_ETH_DEST_ADDR_1    14
#define FPGA_SPI_REG_ETH_DEST_ADDR_2    15
#define FPGA_SPI_REG_CAM_MAX_ROWS       16
#define FPGA_SPI_REG_FINGER_BAUD_DIV    17
#define FPGA_SPI_REG_RS485_UDP_TX       18

#define FPGA_SPI_WRITE                0x80

void fpga_init();
uint16_t fpga_spi_txrx(uint8_t reg, uint16_t tx_data);
uint8_t fpga_is_init_complete();
void fpga_start_configuration();

#endif

