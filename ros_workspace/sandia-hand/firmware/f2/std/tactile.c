#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "tactile.h"
#include "pins.h"

uint16_t g_tactile_last_scan[TACTILE_NUM_TAXELS];
uint16_t g_tactile_current_scan[TACTILE_NUM_TAXELS];
static volatile uint32_t g_tactile_tc0_ovf_count = 0;

static void tactile_strain_adc_reset();
static void tactile_strain_adc_write_reg(uint8_t reg_idx, uint32_t reg_val);

void tactile_init()
{
  PMC_EnablePeripheral(ID_TC0);
  TC0->TC_QIDR = 0xffffffff; // no quadrature interrupts plz
  TC0->TC_CHANNEL[0].TC_IDR = 0xffffffff; // no timer interrupts
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_COVFS; // enable overflow interrupt
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | // 64 / 128 = 500khz
                              TC_CMR_WAVE; // waveform generation mode
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;
  NVIC_SetPriority(TC0_IRQn, 2);
  NVIC_EnableIRQ(TC0_IRQn);

  PMC_EnablePeripheral(ID_SPI);
  PIO_Configure(&pin_cs_adc_tactile, 1);
  PIO_Configure(&pin_cs_adc_strain, 1);
  PIO_Configure(&pin_spi_sck, 1);
  PIO_Configure(&pin_spi_mosi, 1);
  PIO_Configure(&pin_spi_miso, 1);
  PIO_Configure(pin_leds, TACTILE_NUM_TAXELS);
  SPI->SPI_CR = SPI_CR_SPIDIS;
  SPI->SPI_CR = SPI_CR_SWRST;
  SPI->SPI_CR = SPI_CR_SWRST;
  SPI->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;
  SPI->SPI_IDR = 0xffffffff;
  // set scbr to F_CPU / F_SPI
  SPI->SPI_CSR[0] = SPI_CSR_BITS_16_BIT | SPI_CSR_SCBR(5) | 
                    SPI_CSR_NCPHA; // | SPI_CSR_CPOL;*/
  SPI->SPI_CR = SPI_CR_SPIEN;
  volatile uint32_t dummy;
  for (dummy = 0; dummy < 100000; dummy++) { } // why? atmel does it
  dummy = REG_SPI_SR;
  dummy = REG_SPI_RDR;
  for (int i = 0; i < TACTILE_NUM_TAXELS; i++)
    g_tactile_last_scan[i] = g_tactile_current_scan[i] = 0;
  //tactile_strain_adc_reset();
}

typedef struct tactile_map
{
  uint8_t led_chan;
  uint8_t adc_chan;
} tactile_map_t;

void tactile_tc0_irq()
{
  g_tactile_tc0_ovf_count++; 
  TC0->TC_CHANNEL[0].TC_SR; // dummy read
}

const tactile_map_t tactile_map[TACTILE_NUM_TAXELS] =
{
  { 3, 5}, // emit_0_0: led 3, adc ch 5
  { 0, 4}, // emit_0_1: led 0, adc ch 4
  { 4, 3}, // emit_1_0: led 4, adc ch 3
  { 1, 2}, // emit_1_1: led 1, adc ch 2
  { 5, 7}, // emit_2_0: led 5, adc ch 7
  { 2, 6}, // emit_2_1: led 2, adc ch 6
};

/*
static uint16_t adc_scan_buf[3][9];
void scan_adcs()
{
  for (adc_idx = 0; adc_idx < 3; adc_idx++)
  {
    PIO_Clear(&pin_cs_adc[chip_idx]);
    uint16_t cfg = 0x3
    PIO_Set(&pin_cs_adc[chip_idx]);
  }
}
*/

#define ADC_T_EN { }
#define ADC_T_DIS { }
//#define ADC_T_EN { for (volatile int i = 0; i < 10; i++) { } }
//#define ADC_T_DIS { for (volatile int i = 0; i < 10; i++) { } }
#define ADC_T_CYC { for (volatile int i = 0; i < 10; i++) { } }

void set_pin(const Pin *pin, const uint8_t state)
{
  if (state)
    PIO_Set(pin);
  else
    PIO_Clear(pin);
}

uint16_t tactile_read_adc(uint8_t adc_chan, const uint8_t num_samples)
{
  // 0x3c49
  const uint16_t adc_cfg = (0x3c09 | (adc_chan << 7)) << 2;
  volatile uint16_t rx;
  uint32_t adc_sum = 0;

  for (int i = 0; i < 3 + num_samples; i++)
  {
    PIO_Clear(&pin_cs_adc_tactile);
    ADC_T_EN;
    SPI->SPI_TDR = adc_cfg;
    while ((SPI->SPI_SR & SPI_SR_TXEMPTY) == 0) { }
    rx = SPI->SPI_RDR;
    ADC_T_DIS;
    PIO_Set(&pin_cs_adc_tactile);
    ADC_T_CYC;
    if (i >= 3)
    {
      adc_sum += rx;
      for (volatile int j = 0; j < 100; j++) { }
    }
  }
  return adc_sum / num_samples;
}
#if 0
void tactile_scan(uint8_t *data_buf, const uint8_t with_leds)
{
  for (int sensor_idx = 0; sensor_idx < TACTILE_NUM_TAXELS; sensor_idx++)
  {
    const tactile_map_t *map = &tactile_map[sensor_idx];
    __disable_irq();
    if (with_leds)
      PIO_Clear(&pin_leds[map->led_chan]); // turn LED on plz
    uint16_t adc_reading = read_tactile_adc(map->adc_chan, 1); // switch chan
    for (volatile int32_t i = 0; i < 200; i++) { } // wait for settling 
    adc_reading = read_tactile_adc(map->adc_chan, 10);
    if (with_leds)
      PIO_Set(&pin_leds[map->led_chan]); // turn LED off plz
    __enable_irq();
    *((uint16_t *)(data_buf + sensor_idx*2)) = adc_reading;
  }
}
#endif
void tactile_idle()
{
}

static volatile uint8_t g_tactile_sensor_idx = 0;
static volatile uint8_t g_tactile_sensor_state_count = 0; // counts how many systicks
static const uint8_t TACTILE_SETTLE_SYSTICKS = 0;

void tactile_systick()
{
  //if (++g_tactile_sensor_state_count < TACTILE_SETTLE_SYSTICKS)
  //  return; // move along. nothing to see here.
  // it's time to scan this guy. do a bunch of ADC readings, decimate (someday)
  //g_tactile_last_scan[0]++;
  g_tactile_current_scan[g_tactile_sensor_idx] = 
              tactile_read_adc(tactile_map[g_tactile_sensor_idx].adc_chan, 10);
  PIO_Set(&pin_leds[tactile_map[g_tactile_sensor_idx].led_chan]); // turn off
  // increment (with wraparound) to find out WHO'S NEXT
  g_tactile_sensor_idx++;
  if (g_tactile_sensor_idx >= TACTILE_NUM_TAXELS)
  {
    g_tactile_sensor_idx = 0;
    for (int i = 0; i < TACTILE_NUM_TAXELS; i++)
      g_tactile_last_scan[i] = g_tactile_current_scan[i]; // batch update
  }
  PIO_Clear(&pin_leds[tactile_map[g_tactile_sensor_idx].led_chan]); // turn on
  g_tactile_sensor_state_count = 0; // reset the counter
}
#if 0
void tactile_strain_adc_reset()
{
  PIO_Clear(&pin_cs_adc_strain);
  for (int i = 0; i < 10; i++)
  {
    SPI->SPI_TDR = 0xff; // send it lots of 0xff to reset it
    while ((SPI->SPI_SR & SPI_SR_TXEMPTY) == 0) { } // busy wait
    SPI->SPI_RDR; // flush rx buffer 
  }
  PIO_Set(&pin_cs_adc_strain);
  for (volatile int i = 0; i < 100000; i++) { }
}

//#define TACTILE_STRAIN_WAIT_FOR_DOUT() for (volatile int i = 0; i < 5000000 && (PIOA->

void tactile_strain_adc_write_reg(uint8_t reg_idx, uint32_t reg_val)
{
  int bytes_wide = 3;
  if (reg_idx == 0 ||
      reg_idx == 4 ||
      reg_idx == 5)
    bytes_wide = 1; // weird interface.
  PIO_Clear(&pin_cs_adc_strain);
  for (volatile int i = 0; i < 5; i++) { }
  WAIT_FOR_DOUT();
  SPI->SPI_TDR = ((reg_idx & 0x7) << 3);
  while ((SPI->SPI_SR & SPI_SR_TXEMPTY) == 0) { } // wait for txrx
  SPI->SPI_RDR;
  for (int i = 0; i < bytes_wide; i++)
  {
    SPI->SPI_TDR = 0xff & (reg_val >> ((bytes_wide - i - 1) * 8));
    while ((SPI->SPI_SR & SPI_SR_TXEMPTY) == 0) { } // wait for txrx
    SPI->SPI_RDR;
  }
  PIO_Set(&pin_cs_adc_strain);
}
#endif

#if 0
void thermal_scan(uint8_t *data_buf)
{
  for (int therm_idx = 0; therm_idx < NUM_THERM; therm_idx++)
  {
    const adc_mapping_t *mapping = &therm_mappings[therm_idx];
    __disable_irq();
    // set the muxes
    set_pin(&pin_mux[0][0], mapping->mux0_chan & 0x01);
    set_pin(&pin_mux[0][1], mapping->mux0_chan & 0x02);
    set_pin(&pin_mux[0][2], mapping->mux0_chan & 0x04);
    set_pin(&pin_mux[1][0], mapping->mux1_chan & 0x01);
    set_pin(&pin_mux[1][1], mapping->mux1_chan & 0x02);
    set_pin(&pin_mux[1][2], mapping->mux1_chan & 0x04);
    uint16_t adc_reading = read_adc(mapping->adc_chan, 1);
    for (volatile int i = 0; i < 2000; i++) { } // wait for settling 
    adc_reading = read_adc(mapping->adc_chan, 10);
    __enable_irq();
    *((uint16_t *)(data_buf + therm_idx*2)) = adc_reading;
  }
}
#endif

