#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "state.h"
#include "tactile.h"
#include <stdbool.h>
#include "imu.h"

static volatile uint32_t g_state_tc0_ovf_count = 0;
static bool g_state_build_packet = false;
palm_state_t g_state;
void state_init()
{
  for (int i = 0; i < sizeof(palm_state_t); i++)
    ((uint8_t *)(&g_state))[i] = 0; // ugly
  PMC_EnablePeripheral(ID_TC0);
  TC0->TC_CHANNEL[0].TC_IDR = 0xffffffff; // no timer interrupts
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_COVFS; // enable overflow interrupt
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | // 64 / 128 = 500khz
                              TC_CMR_WAVE; // waveform generation mode
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
  NVIC_SetPriority(TC0_IRQn, 4);
  NVIC_EnableIRQ(TC0_IRQn);
}

void state_tc0_irq()
{
  g_state_tc0_ovf_count++;
  TC0->TC_CHANNEL[0].TC_SR; // dummy read to clear IRQ flag
}

void state_systick()
{
  static volatile int s_state_systick_count = 0;
  if (s_state_systick_count++ % (1000 / 100) == 0) // 100 hz
    g_state_build_packet = true;
}

void state_idle()
{
  if (!g_state_build_packet)
    return;
  g_state_build_packet = false;
  // stuff the state buffer, so that it's ready for pickup by mobo autopoll
  for (int i = 0; i < TACTILE_NUM_TAXELS; i++)
    g_state.palm_tactile[i] = g_tactile_last_scan[i];
  for (int i = 0; i < 7; i++)
    g_state.palm_temps[i] = 0; // todo
  for (int i = 0; i < 3; i++)
  {
    g_state.palm_accel[i] = g_imu_data[i];
    g_state.palm_gyro[i]  = 0;
    g_state.palm_mag[i]   = g_imu_data[i+3];
  }
  g_state.palm_time = g_tactile_last_scan_time;
}

uint32_t state_get_time()
{
  return (g_state_tc0_ovf_count << 17) + // want microseconds
         ((uint32_t)TC0->TC_CHANNEL[0].TC_CV << 1); 
}

