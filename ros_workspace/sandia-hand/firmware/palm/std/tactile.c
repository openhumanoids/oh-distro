#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "tactile.h"
#include "pins.h"
#include "state.h"

#define TACTILE_NUM_MUXES 5

uint16_t g_tactile_last_scan[TACTILE_NUM_TAXELS];
uint32_t g_tactile_last_scan_time = 0;
uint16_t g_tactile_current_scan[TACTILE_NUM_TAXELS];


typedef struct
{
  uint8_t adc_idx;
  uint8_t adc_chan;
} mux_t;

mux_t muxes[TACTILE_NUM_MUXES] =
{
  {0, 5},
  {1, 4},
  {0, 4},
  {1, 5},
  {0, 0},
};

typedef struct 
{
  uint8_t led_idx;
  uint8_t mux_idx;
  uint8_t mux_chan;
} photosensor_t;

// LED array offsets are defined in pins.c
photosensor_t photosensors[TACTILE_NUM_TAXELS] =
{
  {0, 0, 4}, // emit_0_0 , leds_0, mux 0 ch 4, adc 0 ch 5
  {0, 0, 7}, // emit_0_1 , leds_0, mux 0 ch 7, adc 0 ch 5
  {1, 0, 6}, // emit_1_0 , leds_1, mux 0 ch 6, adc 0 ch 5
  {1, 0, 3}, // emit_1_1 , leds_1, mux 0 ch 3, adc 0 ch 5
  {2, 3, 2}, // emit_2_0 , leds_2, mux 3 ch 2, adc 1 ch 4
  {2, 1, 6}, // emit_2_1 , leds_2, mux 1 ch 6, adc 
  {3, 1, 5}, // emit_3_0 , leds_3, mux 1 ch 5,
  {3, 1, 7}, // emit_3_1 , leds_3, mux 1 ch 7,
  {4, 0, 2}, // emit_4_0 , leds_4, mux 0 ch 2,
  {4, 2, 5}, // 9 emit_4_1 , leds_4, mux 2 ch 5,
  {5, 1, 3}, // emit_5_0 , leds_5, mux 1 ch 3,
  {5, 1, 2}, // 11 emit_5_1 , leds_5, mux 1 ch 2,
  {6, 2, 6}, // 12 emit_6_0 , leds_6, mux 2 ch 6,
  {6, 0, 1}, // 13 emit_6_1 , leds_6, mux 0 ch 1,
  {7, 2, 0}, // 14 emit_7_0 , leds_7, mux 2 ch 0,
  {7, 2, 7}, // 15 emit_7_1 , leds_7, mux 2 ch 7,
  {8, 2, 1}, // 16 emit_8_0 , leds_8, mux 2 ch 1,
  {8, 0, 0}, // 17 emit_8_1 , leds_8, mux 0 ch 0,
  {9, 2, 3}, // 18 emit_9_0 , leds_9, mux 2 ch 3,
  {9, 2, 4}, // 19 emit_9_1 , leds_9, mux 2 ch 4,
  {10, 4, 4}, // emit_10_0, leds_10, mux 4 ch 4,
  {10, 1, 1}, // emit_10_1, leds_10, mux 1 ch 1,
  {11, 4, 0}, // emit_11_0, leds_11_0, mux 4 ch 0,
  {12, 4, 5}, // emit_11_1, leds_11_1, mux 4 ch 5,
  {13, 1, 0}, // emit_12_0, leds_12_0, mux 1 ch 0,
  {14, 3, 0}, // emit_12_1, leds_12_1, mux 3 ch 0,
  {15, 3, 4}, // emit_13_0, leds_13_0, mux 3 ch 4,
  {16, 3, 1}, // emit_13_1, leds_13_1, mux 3 ch 1,
  {17, 3, 7}, // emit_14_0, leds_14, mux 3 ch 7,
  {17, 3, 3}, // emit_14_1, leds_14, mux 3 ch 3,
  {18, 3, 5}, // emit_15_0, leds_15_0, mux 3 ch 5,
  {19, 4, 2}, // emit_15_1, leds_15_1, mux 4 ch 2,
};

/*  todo: fix this
const sensor_map_t therm_mappings[TACTILE_NUM_THERM] =
{
  { 0, 5, 0, 0 },
  { 0, 4, 0, 0 },
  { 1, 0, 2, 0 },
  { 1, 0, 7, 0 },
};
*/
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

void set_mux(mux_t *mux, uint8_t adc_idx, uint8_t adc_chan)
{
  mux->adc_idx  = adc_idx;
  mux->adc_chan = adc_chan;
}

void set_photosensor(photosensor_t *p, 
                     uint8_t led_idx, uint8_t mux_idx, uint8_t mux_chan)
{
  p->led_idx  = led_idx;
  p->mux_idx  = mux_idx;
  p->mux_chan = mux_chan;
}

void tactile_init()
{
  PMC_EnablePeripheral(ID_SPI);
  PIO_Configure(pin_cs_adc, PINS_NUM_CS_ADC);
  PIO_Configure(&pin_spi_sck, 1);
  PIO_Configure(&pin_spi_mosi, 1);
  PIO_Configure(&pin_spi_miso, 1);
  PIO_Configure(pin_leds, PINS_NUM_LEDS);
  for (int i = 0; i < PINS_NUM_MUX; i++)
    PIO_Configure(pin_mux[i], PINS_MUX_ADDR_BITS);
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
  if (g_pins_hand == 'L')
  {
    set_mux(&muxes[0], 0, 4);
    set_mux(&muxes[1], 1, 3);
    set_mux(&muxes[2], 0, 5);
    set_mux(&muxes[3], 1, 2);
    set_mux(&muxes[4], 0, 7);
    set_photosensor(&photosensors[0] ,  0, 0, 5);
    set_photosensor(&photosensors[1] ,  0, 0, 7);
    set_photosensor(&photosensors[2] ,  1, 0, 6);
    set_photosensor(&photosensors[3] ,  1, 0, 3);
    set_photosensor(&photosensors[4] ,  2, 3, 5); // emit_2_0
    set_photosensor(&photosensors[5] ,  2, 1, 6);
    set_photosensor(&photosensors[6] ,  3, 1, 5);
    set_photosensor(&photosensors[7] ,  3, 1, 7);
    set_photosensor(&photosensors[8] ,  4, 0, 2); // emit_4_0
    set_photosensor(&photosensors[9] ,  4, 2, 1);
    set_photosensor(&photosensors[10],  5, 1, 3);
    set_photosensor(&photosensors[11],  5, 1, 2); 
    set_photosensor(&photosensors[12],  6, 2, 2); // emit_6_0
    set_photosensor(&photosensors[13],  6, 0, 1);
    set_photosensor(&photosensors[14],  7, 2, 4);
    set_photosensor(&photosensors[15],  7, 2, 7);
    set_photosensor(&photosensors[16],  8, 2, 6); // emit_8_0
    set_photosensor(&photosensors[17],  8, 0, 4);
    set_photosensor(&photosensors[18],  9, 2, 3);
    set_photosensor(&photosensors[19],  9, 2, 0);
    set_photosensor(&photosensors[20], 10, 4, 0); // emit_10_0
    set_photosensor(&photosensors[21], 10, 1, 1);
    set_photosensor(&photosensors[22], 11, 4, 4); // emit_11_0
    set_photosensor(&photosensors[23], 12, 4, 1);
    set_photosensor(&photosensors[24], 13, 1, 0); // emit_12_0
    set_photosensor(&photosensors[25], 14, 3, 0);
    set_photosensor(&photosensors[26], 15, 3, 1); // emit_13_0
    set_photosensor(&photosensors[27], 16, 3, 4);
    set_photosensor(&photosensors[28], 17, 3, 7);
    set_photosensor(&photosensors[29], 17, 3, 6);
    set_photosensor(&photosensors[30], 18, 3, 2); // emit_15_0
    set_photosensor(&photosensors[31], 19, 4, 6);
  }
}

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

uint16_t tactile_read_adc(const uint8_t adc_idx, const uint8_t adc_chan, 
                          const uint8_t num_samples)
{
  // 0x3c49
  const uint16_t adc_cfg = (0x3c09 | (adc_chan << 7)) << 2;
  volatile uint16_t rx;
  uint32_t adc_sum = 0;

  for (int i = 0; i < 3 + num_samples; i++)
  {
    PIO_Clear(&pin_cs_adc[adc_idx]);
    ADC_T_EN;
    SPI->SPI_TDR = adc_cfg;
    while ((SPI->SPI_SR & SPI_SR_TXEMPTY) == 0) { }
    rx = SPI->SPI_RDR;
    ADC_T_DIS;
    PIO_Set(&pin_cs_adc[adc_idx]);
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
#if 0
  //PIO_Toggle(&pin_leds[18]);
  //pin_leds[18].pio->PIO_TODR = pin_leds[18].mask;
  if (++g_tactile_sensor_state_count % 2 == 0)
    PIOC->PIO_SODR = PIO_PC18;
    //PIO_Set(&pin_leds[18]);
  else
    PIOC->PIO_CODR = PIO_PC18;
    //PIO_Clear(&pin_leds[18]);
  return;
#endif

  // since we are reading a tactile sensor each systick, no need for this check
  //if (++g_tactile_sensor_state_count < TACTILE_SETTLE_SYSTICKS)
  //  return; // move along. nothing to see here.

  // it's time to scan this guy. do a bunch of ADC readings, someday decimate 
  const photosensor_t *ps_prev = &photosensors[g_tactile_sensor_idx];
  const mux_t *mux_prev = &muxes[ps_prev->mux_idx];
  g_tactile_current_scan[g_tactile_sensor_idx] =
              tactile_read_adc(mux_prev->adc_idx, mux_prev->adc_chan, 10);
  PIO_Set(&pin_leds[ps_prev->led_idx]); // turn off LED plz
  // increment (with wraparound) to find out WHO'S NEXT
  g_tactile_sensor_idx++;
  if (g_tactile_sensor_idx >= TACTILE_NUM_TAXELS)
  {
    g_tactile_sensor_idx = 0;
    for (int i = 0; i < TACTILE_NUM_TAXELS; i++)
      g_tactile_last_scan[i] = g_tactile_current_scan[i]; // batch update plz
    g_tactile_last_scan_time = state_get_time();
  }
  // now, move on to the next photosensor
  const photosensor_t *ps_next = &photosensors[g_tactile_sensor_idx];
  //const mux_t *mux_next = &muxes[ps_next->mux_idx]; // unneeded....
  PIO_Clear(&pin_leds[ps_next->led_idx]); // turn on LED plz
  // set the mux address lines of the activated photosensor
  for (int i = 0; i < 3; i++)
    set_pin(&pin_mux[ps_next->mux_idx][i], ps_next->mux_chan & (1 << i));
  g_tactile_sensor_state_count = 0; // reset the counter. don't need to tho.
  // now, everything is settling. wait until the next systick fires, at which
  // time we shall sample it a few times and call it good.
}

