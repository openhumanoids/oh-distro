#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "tactile.h"
#include "pins.h"

uint16_t g_tactile_last_scan[TACTILE_NUM_TAXELS];
uint16_t g_tactile_current_scan[TACTILE_NUM_TAXELS];

void tactile_init()
{
  PMC_EnablePeripheral(ID_SPI);
  PIO_Configure(&pin_cs_adc, 1);
  PIO_Configure(&pin_spi_sck, 1);
  PIO_Configure(&pin_spi_mosi, 1);
  PIO_Configure(&pin_spi_miso, 1);
  PIO_Configure(pin_leds, 6);
  PIO_Configure(pin_mux[0], 3);
  PIO_Configure(pin_mux[1], 3);
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
}

typedef struct 
{
  uint8_t adc_chan;
  uint8_t mux0_chan;
  uint8_t mux1_chan;
  uint8_t led_chan;
} sensor_map_t;

// goal of this table is to minimize the number of mux switches
const sensor_map_t tactile_map[TACTILE_NUM_TAXELS] =
{
  { 0, 3, 0, 0 }, // emit_0_0: mux0 ch3
  { 1, 3, 5, 0 }, // emit_0_1: mux1 ch5
  { 0, 7, 5, 1 }, // emit_1_0: mux0 ch7
  { 0, 6, 5, 1 }, // emit_1_1: mux0 ch6
  { 0, 0, 3, 2 }, // emit_2_0: mux0 ch0
  { 1, 0, 3, 2 }, // emit_2_1: mux1 ch3
  { 0, 1, 3, 3 }, // emit_3_0: mux0 ch1
  { 1, 1, 6, 3 }, // emit_3_1: mux1 ch6
  { 1, 1, 0, 4 }, // emit_4_0: mux1 ch0
  { 1, 1, 1, 4 }, // emit_4_1: mux1 ch1
  { 1, 1, 4, 5 }, // emit_5_0: mux1 ch4
  { 0, 2, 4, 5 }, // emit_5_1: mux0 ch2
};

const sensor_map_t therm_mappings[TACTILE_NUM_THERM] =
{
  { 0, 5, 0, 0 },
  { 0, 4, 0, 0 },
  { 1, 0, 2, 0 },
  { 1, 0, 7, 0 },
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
    PIO_Clear(&pin_cs_adc);
    ADC_T_EN;
    SPI->SPI_TDR = adc_cfg;
    while ((SPI->SPI_SR & SPI_SR_TXEMPTY) == 0) { }
    rx = SPI->SPI_RDR;
    ADC_T_DIS;
    PIO_Set(&pin_cs_adc);
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
  for (int sensor_idx = 0; sensor_idx < NUM_SENSORS; sensor_idx++)
  {
    const adc_mapping_t *mapping = &adc_mappings[sensor_idx];
    __disable_irq();
    if (sensor_idx % 2 == 0 && with_leds)
      PIO_Clear(&pin_leds[mapping->led_chan]); // turn light on plz
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
    if (sensor_idx % 2 == 1 && with_leds)
      PIO_Set(&pin_leds[mapping->led_chan]); // turn off light
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
  const sensor_map_t *m = &tactile_map[g_tactile_sensor_idx];
  PIO_Clear(&pin_leds[m->led_chan]); // turn on
  // set the muxes
  set_pin(&pin_mux[0][0], m->mux0_chan & 0x01);
  set_pin(&pin_mux[0][1], m->mux0_chan & 0x02);
  set_pin(&pin_mux[0][2], m->mux0_chan & 0x04);
  set_pin(&pin_mux[1][0], m->mux1_chan & 0x01);
  set_pin(&pin_mux[1][1], m->mux1_chan & 0x02);
  set_pin(&pin_mux[1][2], m->mux1_chan & 0x04);
  g_tactile_sensor_state_count = 0; // reset the counter. don't need to tho.
  // then everything will settle until the next systick fires, at which
  // point we sample it a few times and call it good.
}

// todo: schedule this reasonably.
void thermal_scan(uint8_t *data_buf)
{
  for (int therm_idx = 0; therm_idx < TACTILE_NUM_THERM; therm_idx++)
  {
    const sensor_map_t *mapping = &therm_mappings[therm_idx];
    __disable_irq();
    // set the muxes
    set_pin(&pin_mux[0][0], mapping->mux0_chan & 0x01);
    set_pin(&pin_mux[0][1], mapping->mux0_chan & 0x02);
    set_pin(&pin_mux[0][2], mapping->mux0_chan & 0x04);
    set_pin(&pin_mux[1][0], mapping->mux1_chan & 0x01);
    set_pin(&pin_mux[1][1], mapping->mux1_chan & 0x02);
    set_pin(&pin_mux[1][2], mapping->mux1_chan & 0x04);
    uint16_t adc_reading = tactile_read_adc(mapping->adc_chan, 1);
    for (volatile int i = 0; i < 2000; i++) { } // wait for settling 
    adc_reading = tactile_read_adc(mapping->adc_chan, 10);
    __enable_irq();
    *((uint16_t *)(data_buf + therm_idx*2)) = adc_reading;
  }
}

