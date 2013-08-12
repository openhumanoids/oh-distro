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
#include "enet.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "flash.h"
#include "led.h"
#include "boot.h"
#include "config.h"

volatile uint32_t systick_count = 0;

void systick_vector()
{
  enet_systick();
  systick_count++;
}

void main()
{
  config_init();
  console_init();
  printf("bootloader main()\r\n");
  const int MAX_CONFIG_ATTEMPTS = 3;
  int attempt = 0;
  for (; !fpga_is_init_complete() && attempt < MAX_CONFIG_ATTEMPTS; attempt++)
  {
    fpga_init();
    flash_init(); // if needed, blow away fpga image with golden one.
  }
  if (attempt >= MAX_CONFIG_ATTEMPTS)
  {
    printf("fpga will not configure. now i will go into infinite loop...\r\n");
    while (1) { } // aaahhhhhhhhhhhhhhhhhhhhh
  }
  enet_init();
  printf("bootloader! hello, world!\r\n");
  __enable_irq();
  fpga_spi_txrx(0x80, 1); // turn off fpga led
  // reset the PHY via hardware reset pin
  //printf("asserting PHY hardware reset\r\n");
  fpga_spi_txrx(FPGA_SPI_REG_MDIO_CFG | FPGA_SPI_WRITE, 4); // assert PHY_RESET
  for (volatile int j = 0; j < 2000000; j++) { } // wait a while
  fpga_spi_txrx(FPGA_SPI_REG_MDIO_CFG | FPGA_SPI_WRITE, 0); // release PHY_RESET
  for (volatile int j = 0; j < 2000000; j++) { } // wait a longer while
  //printf("requesting PHY software reset\r\n");
  // now do a software reset of the PHY 
  fpga_spi_txrx(FPGA_SPI_REG_MDIO_WDATA | FPGA_SPI_WRITE, 
                0x9000); // set SW reset bit and auto-negotiate bit
  fpga_spi_txrx(FPGA_SPI_REG_MDIO_CFG | FPGA_SPI_WRITE,
                0x0001); // start write of register zero
  for (volatile int j = 0; j < 2000000; j++) { } // wait a longer while
  SysTick_Config(F_CPU/1000); // set up 1 khz systick

  if ((*(uint32_t *)0x000088000) == 0)
  {
    printf("boot not possible; application vector table is undefined.\r\n");
    boot_enabled = 0; // impossible to boot. don't time out.
  }

  printf("waiting until we have ARP to 10.10.1.1 ...\r\n");
  // spin here until we have ARP, or until 20 seconds expire
  uint32_t start_time = systick_count;
  uint32_t dance_time = systick_count;
  for (uint32_t loop_count = 0; !enet_arp_valid(); loop_count++)
  {
    enet_idle(); 
    if (systick_count != dance_time && systick_count % 50 == 0)
    {
      dance_time = systick_count;
      led_dance();
    }
    if (systick_count - start_time >= 15000)
      break; // didn't hear back from ARP. sad. time to give up and move on.
  }

  printf("entering bootloader wait loop\r\n");
  start_time = systick_count;
  for (uint32_t loop_count = 0; ; loop_count++)
  {
    enet_idle();
    if (systick_count != dance_time && systick_count % 100 == 0)
    {
      dance_time = systick_count;
      led_dance();
    }
    if ((systick_count - start_time >= 5000 && boot_enabled) ||
        boot_requested)
      break; // hit timeout. boot.
  }
  printf("jumping to application. bye...\r\n");
  typedef uint32_t (*app_fp)();
  app_fp app = *((app_fp *)0x88004); // look up application start address
  app(); // and call it
  while (1) { } // shouldn't return, but if we do, hang out here.
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

