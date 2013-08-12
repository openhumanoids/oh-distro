#include "common_sam3x/sam3x.h"
#include <stdio.h>
#include "flash.h"
#include "fpga.h"

// xilinx user guide seems to say that it takes 19,719,712 bits to configure
// a spartan6 LX75, but the bin file for the prom seems to be slightly smaller.
// not sure why.

// hardware connections:
//  PA25 = mcu_miso
//  PA26 = mcu_mosi
//  PA27 = mcu_sck
//  PA29 = mcu_flash_cs
//  PC29 = flash_sel
void flash_spi_txrx(const uint8_t instr, const uint16_t data_len, 
                    const uint8_t *tx_data, uint8_t *rx_data,
                    const uint8_t rx_ignore_bytes);

void flash_init()
{
  printf("flash_init()\r\n");
  PMC->PMC_PCER0 |= (1 << ID_PIOC);
  PIOC->PIO_PER = PIOC->PIO_OER = PIOC->PIO_CODR = PIO_PC29; // low = fpga ctrl
  // NOTE! this assumes that fpga_spi_init() has already run, which set up
  // the SPI controller and pins.
  if (!fpga_is_init_complete())
  {
    uint8_t page_buf[256] = {0};
    printf("fpga did not configure. replacing root image with golden...\r\n");
    for (int page_num = 0; page_num < 10000; page_num++) // 10k for xc6slx75
    {
      flash_read_page(32768 + page_num, page_buf);
      if ((page_num % 256) == 0) // first page of this sector; erase first.
      {
        if ((page_num % (256*16)) == 0)
          printf("erasing sector 0x%x  (page 0x%x)\r\n", 
                 page_num/256, page_num);
        flash_erase_sector(page_num);
      }
      flash_write_page(page_num, page_buf);
    }
    printf("done replacing root image. starting fpga init cycle...\r\n");
    fpga_start_configuration();
  }
}

void flash_spi_txrx(const uint8_t instr, const uint16_t data_len, 
                    const uint8_t *tx_data, uint8_t *rx_data,
                    const uint8_t rx_ignore_bytes)
{
  PIOC->PIO_SODR = PIO_PC29; // assert control of the flash SPI lines
  while (!(SPI0->SPI_SR & SPI_SR_TXEMPTY)) { } // ensure peripheral is clear
  //volatile uint16_t rx;
  PIOA->PIO_CODR = PIO_PA29; // drive flash CS line low
  SPI0->SPI_TDR = instr; // send instruction byte
  while (!(SPI0->SPI_SR & SPI_SR_TXEMPTY)) { } // wait for it to shift out
  volatile uint8_t garbage = SPI0->SPI_RDR; // clear the garbage rx
  for (uint32_t i = 0; i < data_len; i++)
  {
    SPI0->SPI_TDR = tx_data ? tx_data[i] : 0; // start txrx
    while (!(SPI0->SPI_SR & SPI_SR_TXEMPTY)) { } // wait for it to shift out
    if (rx_data && i >= rx_ignore_bytes)
      rx_data[i - rx_ignore_bytes] = SPI0->SPI_RDR & 0xff; // copy out rx byte
    else
      SPI0->SPI_RDR;
  }
  PIOA->PIO_SODR = PIO_PA29; // drive flash CS line high
  PIOC->PIO_CODR = PIO_PC29; // release control of the flash SPI lines
}

uint8_t flash_read_status_register()
{
  uint8_t rx_data = 0;
  flash_spi_txrx(0x05, 1, NULL, &rx_data, 0);
  return rx_data;
}

void flash_read_page(const uint32_t page_num, uint8_t *page_data)
{ 
  // assume 256-byte read pages
  uint8_t tx_data[256+3] = {0};
  tx_data[0] = (page_num >> 8) & 0xff; // page number MSB
  tx_data[1] = page_num & 0xff; // page number LSB
  tx_data[2] = 0; // page-aligned reads
  flash_spi_txrx(0x03, 256+3, tx_data, page_data, 3); // Read Data instruction
  /*
  if (page_num == 32768)
  {
    printf("read page %d...\r\n", page_num);
    for (int i = 0; i < 256; i++) 
    { 
      printf("0x%02x  ", page_data[i]); 
      if (i % 8 == 7) 
        printf("\r\n"); 
    }
  }
  */
}

void flash_write_page(const uint32_t page_num, const uint8_t *page_data)
{ 
  /*
  printf("writing page %d...\r\n", page_num);
  for (int i = 0; i < 256; i++) 
  { 
    printf("0x%02x  ", page_data[i]); 
    if (i % 8 == 7) 
      printf("\r\n"); 
  }
  */
  flash_spi_txrx(0x06, 0, NULL, NULL, 0); // send Write Enable instruction
  uint8_t tx_data[256+3] = {0};
  tx_data[0] = (page_num >> 8) & 0xff; // page number MSB
  tx_data[1] = page_num & 0xff; // page number LSB
  tx_data[2] = 0; // page-aligned writes
  for (int i = 0; i < 256; i++)
    tx_data[i+3] = page_data[i];
  flash_spi_txrx(0x02, 256+3, tx_data, NULL, 0); // Page Program instruction
  // check Write In Progress bit periodicially
  // todo: figure out a decent timeout
  int check_count = 0;
  while (flash_read_status_register() & 0x01) 
  {
    check_count++;
    for (volatile int i = 0; i < 100; i++) { } // burn some cycles
  }
  //printf("write needed %d checks\r\n", check_count);
}

void flash_erase_sector(const uint32_t page_num)
{
  //printf("erasing sector of page %d...\r\n", page_num);
  flash_spi_txrx(0x06, 0, NULL, NULL, 0); // send Write Enable instruction
  uint8_t tx_data[3] = {0};
  tx_data[0] = (page_num >> 8) & 0xff; // page number MSB
  tx_data[1] = page_num & 0xff; // page number LSB
  tx_data[2] = 0; // page-aligned writes
  flash_spi_txrx(0xd8, 3, tx_data, NULL, 0); // Sector Erase instruction
  // check Write In Progress bit periodicially
  // todo: figure out a decent timeout
  int check_count = 0;
  while (flash_read_status_register() & 0x01) 
  {
    check_count++;
    for (volatile int i = 0; i < 100; i++) { } // burn some cycles
  }
  //printf("sector erase needed %d checks\r\n", check_count);
}

