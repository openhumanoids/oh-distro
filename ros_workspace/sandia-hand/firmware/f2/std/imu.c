#include <stdint.h>
#include <stdbool.h>
#include "sam3s/chip.h"
#include "sam3s/pmc.h"
#include "sam3s/twi.h"
#include "sam3s/twid.h"
#include "imu.h"
#include "pins.h"

static Twid g_twid; // twi driver state
static Async g_twid_async; // callback structure. presumably can't be automatic
static bool g_imu_take_measurement = false;
volatile int16_t g_imu_data[6] = {0}; // global buffer
static int16_t g_imu_accel_data[3] = {0}, g_imu_mag_data[3] = {0};
typedef enum { IMU_POWER_ON_REQ , IMU_POWER_ON, IMU_POWER_JUST_ACCEL,
               IMU_POWER_OFF_REQ, IMU_POWER_OFF } imu_power_state_t; 
static imu_power_state_t g_imu_power_state = IMU_POWER_OFF;
#define ACCEL_ADDR 0x19
#define MAG_ADDR 0x1e

#define IMU_MEAS_ACCEL 0
#define IMU_MEAS_MAG   1
#define IMU_MEAS_LAST  1
static uint8_t g_imu_meas = IMU_MEAS_ACCEL;
void imu_twi_cb();

void imu_init()
{
  printf("imu_init()\r\n");
  PMC_EnablePeripheral(ID_TWI0);
  PIO_Configure(&pin_i2c_scl, 1);
  PIO_Configure(&pin_i2c_sda, 1);
  TWI_ConfigureMaster(TWI0, 400000, F_CPU);
  TWID_Initialize(&g_twid, TWI0);
  g_twid_async.status = 0;
  g_twid_async.callback = (void *)imu_twi_cb;
  NVIC_SetPriority(TWI0_IRQn, 3); // lower priority than rs485 comms
  NVIC_EnableIRQ(TWI0_IRQn);
  // write a few registers to init the sensors
  g_imu_power_state = IMU_POWER_OFF;
  imu_reg_write(ACCEL_ADDR, 0x20, 0x00); // low-power mode
  imu_reg_write(ACCEL_ADDR, 0x23, 0x88);
  imu_reg_write(MAG_ADDR  , 0x00, 0x9c); // temp sensor enabled
  imu_reg_write(MAG_ADDR  , 0x01, 0x80); // no idea what range should be
  imu_reg_write(MAG_ADDR  , 0x02, 0x03); // sleep mode
  g_imu_data[0] = 42;
}

void imu_power(uint8_t on)
{
  if (on && (g_imu_power_state == IMU_POWER_ON ||
             g_imu_power_state == IMU_POWER_ON_REQ))
    return;
  if (!on && (g_imu_power_state == IMU_POWER_OFF ||
              g_imu_power_state == IMU_POWER_OFF_REQ))
    return;
  if (on)
    g_imu_power_state = IMU_POWER_ON_REQ;
  else
    g_imu_power_state = IMU_POWER_OFF_REQ;
}

void imu_twi_cb()
{
  if (g_imu_meas == IMU_MEAS_MAG)
  {
    // byte-swap the mag measurements; they're big-endian, unfortunately.
    for (int i = 0; i < 3; i++)
    {
      uint8_t swap = ((uint8_t *)g_imu_mag_data)[2*i];
      ((uint8_t *)g_imu_mag_data)[2*i] = ((uint8_t *)g_imu_mag_data)[2*i+1];
      ((uint8_t *)g_imu_mag_data)[2*i+1] = swap;
    }
    __disable_irq(); // copy into global buffer now
    g_imu_data[3] = g_imu_mag_data[0];
    g_imu_data[4] = g_imu_mag_data[1];
    g_imu_data[5] = g_imu_mag_data[2];
    __enable_irq();
  }
  else if (g_imu_meas == IMU_MEAS_ACCEL)
  {
    // accelerometer measurements are 12-bit masquerading as 16-bit...
    __disable_irq(); // copy into global buffer now
    g_imu_data[0] = g_imu_accel_data[0] >> 4;
    g_imu_data[1] = g_imu_accel_data[1] >> 4;
    g_imu_data[2] = g_imu_accel_data[2] >> 4;
    __enable_irq();
  }
}

void imu_reg_write(uint8_t device, uint8_t reg_addr, uint8_t reg_payload)
{
  TWID_Write(&g_twid, device, reg_addr, 1, &reg_payload, 1, 0); // sync. write
}

uint8_t imu_reg_read(uint8_t device, uint8_t reg_addr)
{
  uint8_t data = 0;
  TWID_Read(&g_twid, device, reg_addr, 1, &data, 1, 0); // synchronous read
  return data;
}

void imu_systick()
{
  static volatile int s_imu_systick_count = 0;
  if (s_imu_systick_count++ % (1000 / 200) == 0) // 100hz for imu+accel
    g_imu_take_measurement = true;
}

void imu_idle()
{
  if (g_imu_take_measurement)
  {
    g_imu_take_measurement = false;
    if (g_imu_power_state == IMU_POWER_ON)
    {
      g_imu_meas = (g_imu_meas + 1) % (IMU_MEAS_LAST + 1);
      if (g_imu_meas == IMU_MEAS_ACCEL)
        TWID_Read(&g_twid, ACCEL_ADDR, 0x28 | 0x80, 1, 
                  (uint8_t *)g_imu_accel_data, 6, &g_twid_async);
      else if (g_imu_meas == IMU_MEAS_MAG)
        TWID_Read(&g_twid, MAG_ADDR  , 0x03, 1, 
                  (uint8_t *)g_imu_mag_data, 6, &g_twid_async);
    }
    else if (g_imu_power_state == IMU_POWER_JUST_ACCEL)
    {
      g_imu_meas = (g_imu_meas + 1) % (IMU_MEAS_LAST + 1);
      if (g_imu_meas == IMU_MEAS_ACCEL)
        TWID_Read(&g_twid, ACCEL_ADDR, 0x28 | 0x80, 1,
                  (uint8_t *)g_imu_accel_data, 6, &g_twid_async);
    }
    else if (g_imu_power_state == IMU_POWER_ON_REQ)
    {
      imu_reg_write(MAG_ADDR  , 0x02, 0x00); // continuous-conversion mode
      imu_reg_write(ACCEL_ADDR, 0x20, 0x57); // turn on all axes
      g_imu_power_state = IMU_POWER_ON;
    }
    else if (g_imu_power_state == IMU_POWER_OFF_REQ)
    {
      imu_reg_write(MAG_ADDR  , 0x02, 0x03); // sleep mode zzzzzzzzz
      //imu_reg_write(ACCEL_ADDR, 0x20, 0x03); // low-power mode
      g_imu_power_state = IMU_POWER_JUST_ACCEL;
    }
  }
}

void imu_twi_irq()
{
  // forward call into the TWID driver
  TWID_Handler(&g_twid);
}

