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

#include "common_sam3x/sam3x.h"
#include "common_sam3x/console.h"
#include "fpga.h"
#include "power.h"
#include "enet.h"
#include "finger.h"
#include "cam.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "flash.h"
#include "control.h"
#include "config.h"

void systick_vector()
{
  power_systick();
  enet_systick();
  finger_systick();
}

void main()
{
  config_init();
  console_init();
  const int MAX_CONFIG_ATTEMPTS = 3;
  int attempt = 0;
  for (; !fpga_is_init_complete() && attempt < MAX_CONFIG_ATTEMPTS; attempt++)
  {
    fpga_init();
    flash_init(); // if needed, blow away fpga image with golden one.
  }
  if (attempt >= MAX_CONFIG_ATTEMPTS)
  {
    //printf("fpga will not configure. now i will go into infinite loop...\r\n");
    while (1) { } // aaahhhhhhhhhhhhhhhhhhhhh
  }
  power_init();
  enet_init();
  finger_init();
  cam_init();
  control_init();
  printf("welcome to mobo mcu. hello, world!\r\n");
  // blink the FPGA LED a few times
  for (int i = 0; i < 4; i++)
  {
    for (volatile int j = 0; j < 200000; j++) { }
    fpga_spi_txrx(0x80, 0);
    for (volatile int j = 0; j < 200000; j++) { }
    fpga_spi_txrx(0x80, 1);
  }
  __enable_irq();
  // reset the PHY via hardware reset pin
  printf("asserting PHY hardware reset\r\n");
  fpga_spi_txrx(FPGA_SPI_REG_MDIO_CFG | FPGA_SPI_WRITE, 4); // assert PHY_RESET
  for (volatile int j = 0; j < 2000000; j++) { } // wait a while
  fpga_spi_txrx(FPGA_SPI_REG_MDIO_CFG | FPGA_SPI_WRITE, 0); // release PHY_RESET
  for (volatile int j = 0; j < 2000000; j++) { } // wait a longer while
  printf("requesting PHY software reset\r\n");
  // now do a software reset of the PHY 
  fpga_spi_txrx(FPGA_SPI_REG_MDIO_WDATA | FPGA_SPI_WRITE, 
                0x9000); // set SW reset bit and auto-negotiate bit
  fpga_spi_txrx(FPGA_SPI_REG_MDIO_CFG | FPGA_SPI_WRITE,
                0x0001); // start write of register zero
  for (volatile int j = 0; j < 2000000; j++) { } // wait a longer while
  printf("entering main loop\r\n");
  /*
  #define TEST_PKT_LEN 60
  const uint16_t test_pkt_len = TEST_PKT_LEN;
  uint8_t test_pkt[TEST_PKT_LEN] = 
  { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // 0-5:  destination MAC
    0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, // 6-11: source MAC
    0x08, 0x06, // 12-13: ethertype: 0806 = ARP
    0x00, 0x01, // 14-15: ARP hardware type = ethernet
    0x08, 0x00, // 16-17: ipv4 protocol type = 0x0800
    0x06, // 18: hardware length = 0x06
    0x04, // 19: protocol length = 0x04
    0x00, 0x01, // 20-21: ARP operation: request
    0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, // 22-27: bogus sender MAC
    0x0a, 0x0a, 0x01, 0x02, // 28-31: sender IP: 10.10.1.2
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 32-37: bogus target MAC
    0x0a, 0x0a, 0x01, 0x01, // 38-41: target IP: 10.10.1.1
    0, 0, 0, 0, 0, 0, // 42-47
    0, 0, 0, 0, 0, 0, // 48-53
    0, 0, 0, 0, 0, 0, // 54-59
  };
  */
  SysTick_Config(F_CPU/1000); // set up 1 khz systick
  cam_init(); // try it again. no idea why camera isn't initializing reliably.

  while (1)
  {
    power_idle();
    enet_idle();
    finger_idle();
    /*
    for (volatile int j = 0; j < 20000000; j++) { }
    printf("tx\r\n");
    enet_tx_raw(test_pkt, test_pkt_len);
    printf("tx avail: %d\r\n", enet_tx_avail());
    while (!enet_tx_avail()) { }
    printf("tx avail now\r\n");
    */
  }
}

///////////////////////////////////////////////////////////////////////////
// libc stub nonsense

extern int _end;
extern caddr_t _sbrk(int incr);

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
  //io_led(true);
  console_send_block((uint8_t *)buf, count);
  //io_led(false);
  return count;
}
extern int _close(int fd) { return -1; }
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

/////////////////////////////////////////////////////////////////////////
// graveyard
#if 0
    for (volatile int j = 0; j < 2000000; j++) { }
    printf("\r\n");
    printf("portb: %08x\r\n", PIOB->PIO_PDSR);
    printf("rsr: %08x\r\n", EMAC->EMAC_RSR);
    printf("ale: %d\r\n", EMAC->EMAC_ALE);
    printf("fro: %d\r\n", EMAC->EMAC_FRO);
    printf("fcs: %d\r\n", EMAC->EMAC_FCSE);
    printf("rre: %d\r\n", EMAC->EMAC_RRE);
    printf("rov: %d\r\n", EMAC->EMAC_ROV);
    printf("rse: %d\r\n", EMAC->EMAC_RSE);
    printf("ele: %d\r\n", EMAC->EMAC_ELE);
    printf("rjb: %d\r\n", EMAC->EMAC_RJA);
    printf("usf: %d\r\n", EMAC->EMAC_USF);
    printf("rle: %d\r\n", EMAC->EMAC_RLE);

/*
  for (int i = 0; i < 4; i++)
  {
    //printf("%d\r\n", i);
    printf("fpga register %d: 0x%04x\r\n", i, fpga_spi_txrx(i, 0));
    for (volatile int j = 0; j < 100000; j++) { }
  }
  // poll the PHY register file
  for (int i = 0; i < 32; i++)
  {
    fpga_spi_txrx(0x81, (uint16_t)((i << 8) | 0x1)); // start read of phy reg
    for (volatile int j = 0; j < 400000; j++) { } // wait a while
    uint16_t reg_val = fpga_spi_txrx(0x02, 0); // read out data
    printf("phy reg %d: 0x%04x\r\n", i, reg_val);
  }
  // poll the extended register file
  for (int i = 256; i < 264; i++)
  {
    for (volatile int j = 0; j < 200000; j++) { }
    fpga_spi_txrx(0x82, 0x0000 | i); // address of extended register
    for (volatile int j = 0; j < 200000; j++) { }
    fpga_spi_txrx(0x81, 0x0b03); // start write of extended register addr
    for (volatile int j = 0; j < 200000; j++) { }
    fpga_spi_txrx(0x81, 0x0d01); // request read of extended register data
    for (volatile int j = 0; j < 200000; j++) { }
    uint16_t reg_val = fpga_spi_txrx(0x02, 0); // read out extended data
    printf("phy ext reg %d: 0x%04x\r\n", i, reg_val);
  }
#if 0 
  // override strap registers for advertising capabilities (mode 3:0)
  // need to set extended register 258 to 0x8001
  for (volatile int j = 0; j < 200000; j++) { }
  fpga_spi_txrx(0x82, 0x8102); // write address 258 = 0x0102, set high bit
  for (volatile int j = 0; j < 200000; j++) { }
  fpga_spi_txrx(0x81, 0x0b03); // start write of extended register addr
  for (volatile int j = 0; j < 200000; j++) { }
  fpga_spi_txrx(0x82, 0x8001); // address 258 desired data = 0x8001
  for (volatile int j = 0; j < 200000; j++) { }
  fpga_spi_txrx(0x81, 0x0c03); // start write of extended register data reg
  for (volatile int j = 0; j < 200000; j++) { }
#endif

  for (volatile int j = 0; j < 4000000; j++) { } // wait a while for negotiate

  // poll the PHY register file
  for (int i = 0; i < 32; i++)
  {
    fpga_spi_txrx(0x81, (uint16_t)((i << 8) | 0x1)); // start read of phy reg
    for (volatile int j = 0; j < 400000; j++) { } // wait a while
    uint16_t reg_val = fpga_spi_txrx(0x02, 0); // read out data
    printf("phy reg %d: 0x%04x\r\n", i, reg_val);
  }
 
  // poll the extended register file
  for (int i = 256; i < 264; i++)
  {
    for (volatile int j = 0; j < 200000; j++) { }
    fpga_spi_txrx(0x82, 0x0000 | i); // address of extended register
    for (volatile int j = 0; j < 200000; j++) { }
    fpga_spi_txrx(0x81, 0x0b03); // start write of extended register addr
    for (volatile int j = 0; j < 200000; j++) { }
    fpga_spi_txrx(0x81, 0x0d01); // request read of extended register data
    for (volatile int j = 0; j < 200000; j++) { }
    uint16_t reg_val = fpga_spi_txrx(0x02, 0); // read out extended data
    printf("phy ext reg %d: 0x%04x\r\n", i, reg_val);
  }
  */

#endif

