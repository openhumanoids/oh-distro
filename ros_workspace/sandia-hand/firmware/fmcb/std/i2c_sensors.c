#include <stdbool.h>
#include "i2c_sensors.h"
#include "sam3s/twid.h"
#include "pins.h"

static Twid g_i2c_sensors_twid; // twi driver state
static Async g_i2c_sensors_twid_async;
volatile uint16_t g_i2c_sensors_data[8];

static const uint8_t SENSOR_MCP9804_TEMP = 0;
static const uint8_t SENSOR_ACCEL = 1;
static const uint8_t SENSOR_MAG = 2;
static const uint8_t SENSOR_LSM303DLHC_TEMP = 3;
static int8_t g_i2c_sensors_whos_next = -1; // must be pronounced in WWE voice
static int8_t g_i2c_sensors_current_txrx = 0;
volatile uint16_t g_i2c_sensors_txrx_buf[3] = {0};
static const uint8_t MCP9804_ADDR = 0x18; // 7-bit address, low bits tied down
static const uint8_t MCP9804_TEMP_REG = 0x05;
static const uint8_t ACCEL_ADDR = 0x19;
static const uint8_t MAG_ADDR = 0x1e;

void i2c_sensors_twi_cb();

void i2c_sensors_reg_write(uint8_t device, uint8_t reg_addr, uint8_t payload)
{
  TWID_Write(&g_i2c_sensors_twid, device, reg_addr, 1, &payload, 1, 0);
}

// this is a _synchronous_ read. only use for initialization!
uint8_t i2c_sensors_reg_read(uint8_t device, uint8_t reg_addr)
{
  uint8_t data = 0;
  TWID_Read(&g_i2c_sensors_twid, device, reg_addr, 1, &data, 1, 0);
  return data;
}

void i2c_sensors_init()
{
  PMC_EnablePeripheral(ID_TWI0);
  PIO_Configure(&pin_i2c_sda, 1);
  PIO_Configure(&pin_i2c_scl, 1);
  TWI_ConfigureMaster(TWI0, 400000, F_CPU);
  TWID_Initialize(&g_i2c_sensors_twid, TWI0);
  g_i2c_sensors_twid_async.status = 0;
  g_i2c_sensors_twid_async.callback = (void *)i2c_sensors_twi_cb; 
  NVIC_SetPriority(TWI0_IRQn, 10); // suuuper low priority
  NVIC_EnableIRQ(TWI0_IRQn);
  i2c_sensors_reg_write(MCP9804_ADDR, 0x08, 0x03);
  i2c_sensors_reg_write(ACCEL_ADDR, 0x20, 0x57);
  i2c_sensors_reg_write(ACCEL_ADDR, 0x23, 0x88);
  i2c_sensors_reg_write(MAG_ADDR  , 0x00, 0x9c);
  i2c_sensors_reg_write(MAG_ADDR  , 0x01, 0x80);
  i2c_sensors_reg_write(MAG_ADDR  , 0x02, 0x00);
}

void i2c_sensors_systick()
{
  static volatile int s_i2c_sensors_systick_count = 0;
  s_i2c_sensors_systick_count++;
  if (g_i2c_sensors_whos_next < 0)
  {
    if (s_i2c_sensors_systick_count % 40 == 0)
      g_i2c_sensors_whos_next = SENSOR_ACCEL;
    if (s_i2c_sensors_systick_count % 40 == 20)
      g_i2c_sensors_whos_next = SENSOR_MAG;
    if (s_i2c_sensors_systick_count % 250 == 10)
      g_i2c_sensors_whos_next = SENSOR_MCP9804_TEMP;
    if (s_i2c_sensors_systick_count % 250 == 15)
      g_i2c_sensors_whos_next = SENSOR_LSM303DLHC_TEMP;
  }
}

void i2c_sensors_idle()
{
  if (g_i2c_sensors_whos_next >= 0)
  {
    if (g_i2c_sensors_whos_next == SENSOR_MCP9804_TEMP)
      TWID_Read(&g_i2c_sensors_twid, MCP9804_ADDR, MCP9804_TEMP_REG, 1, 
                (uint8_t *)&g_i2c_sensors_txrx_buf[0], 2,
                &g_i2c_sensors_twid_async);
    else if (g_i2c_sensors_whos_next == SENSOR_ACCEL)
      TWID_Read(&g_i2c_sensors_twid, ACCEL_ADDR, 0x28 | 0x80, 1,
                (uint8_t *)&g_i2c_sensors_txrx_buf[0], 6,
                &g_i2c_sensors_twid_async);
    else if (g_i2c_sensors_whos_next == SENSOR_MAG)
      TWID_Read(&g_i2c_sensors_twid, MAG_ADDR, 0x03, 1,
                (uint8_t *)&g_i2c_sensors_txrx_buf[0], 6,
                &g_i2c_sensors_twid_async);
    else if (g_i2c_sensors_whos_next == SENSOR_LSM303DLHC_TEMP)
      TWID_Read(&g_i2c_sensors_twid, MAG_ADDR, 0x31, 1,
                (uint8_t *)&g_i2c_sensors_txrx_buf[0], 2,
                &g_i2c_sensors_twid_async);
    g_i2c_sensors_current_txrx = g_i2c_sensors_whos_next;
    g_i2c_sensors_whos_next = -1; // i2c interrupts will handle the rest now
  }
}

void i2c_sensors_twi_irq()
{
  TWID_Handler(&g_i2c_sensors_twid);
}

void i2c_sensors_twi_cb()
{
  __disable_irq();
  if (g_i2c_sensors_current_txrx == SENSOR_MCP9804_TEMP)
    g_i2c_sensors_data[0] = ((g_i2c_sensors_txrx_buf[0] >> 8) & 0xff) |
                            ((g_i2c_sensors_txrx_buf[0] & 0xff) << 8);
  else if (g_i2c_sensors_current_txrx == SENSOR_ACCEL)
    for (int i = 0; i < 3; i++)
      g_i2c_sensors_data[i+1] = ((int16_t)g_i2c_sensors_txrx_buf[i]) >> 4;
  else if (g_i2c_sensors_current_txrx == SENSOR_MAG)
    for (int i = 0; i < 3; i++)
      g_i2c_sensors_data[i+4] = ((g_i2c_sensors_txrx_buf[i] >> 8) & 0xff) |
                                ((g_i2c_sensors_txrx_buf[i] & 0xff) << 8);
  else if (g_i2c_sensors_current_txrx == SENSOR_LSM303DLHC_TEMP)
    g_i2c_sensors_data[7] = g_i2c_sensors_txrx_buf[0];
          //(int16_t)(*(uint8_t *)(&g_i2c_sensors_txrx_buf[0])) << 4;  //((g_i2c_sensors_txrx_buf[0] >> 12) & 0x0f) |
                            //((g_i2c_sensors_txrx_buf[0] & 0x00ff) <<  4);
  __enable_irq();
}

