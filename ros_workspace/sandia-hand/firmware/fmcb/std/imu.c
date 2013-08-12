#include "sam3s/chip.h"
#include "imu.h"
#include "pins.h"

void imu_init()
{
  return;
  // TODO: pull in i2c imu code from f2/f3
#if 0
  volatile uint32_t dummy;
  PMC_EnablePeripheral(ID_SPI);
  PIO_Configure(&pin_accel_mosi, 1);
  PIO_Configure(&pin_accel_miso, 1);
  PIO_Configure(&pin_accel_sck , 1);
  PIO_Configure(&pin_accel_cs  , 1);
  // software reset
  SPI->SPI_CR = SPI_CR_SPIDIS;
  SPI->SPI_CR = SPI_CR_SWRST;
  SPI->SPI_CR = SPI_CR_SWRST;
  SPI->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;
  SPI->SPI_IDR    = 0xffffffff;
  SPI->SPI_CSR[0] = SPI_CSR_CPOL | SPI_CSR_BITS_16_BIT | SPI_CSR_SCBR(64);
  SPI->SPI_CR = SPI_CR_SPIEN;
  for (dummy = 0; dummy < 100000; dummy++) { } // why? atmel library does it.
  dummy = REG_SPI_SR;
  dummy = REG_SPI_RDR;
#endif
}

void imu_accel_reg_write(uint8_t addr, uint8_t val)
{
  /*
  SPI->SPI_TDR = (((uint16_t)addr) << 8) | val;
  while ((SPI->SPI_SR & SPI_SR_TXEMPTY) == 0) { }
  */
}

uint8_t imu_accel_reg_read(uint8_t addr)
{
  return 0x42;
  /*
  SPI->SPI_TDR = ((uint16_t)addr) << 8;
  while ((SPI->SPI_SR & SPI_SR_TXEMPTY) == 0) { }
  return (uint8_t)(SPI->SPI_RDR & 0xff);
  */
}

