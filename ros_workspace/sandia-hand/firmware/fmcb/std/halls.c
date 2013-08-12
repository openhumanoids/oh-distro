#include "halls.h"
#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "pins.h"

volatile int32_t g_hall_count_0, g_hall_count_1, g_hall_count_2;
uint32_t g_hall_0, g_hall_1, g_hall_2, g_hall_prev[3];
volatile int g_hall_state_0, g_hall_state_1, g_hall_state_2;
static int g_hall_decode[8] = {0, 0, 2, 1, 4, 5, 3, 0};
static int g_hall_forward[8] = {1, 2, 3, 4, 5, 0, 0, 0};
static int g_hall_backward[8] = {5, 0, 1, 2, 3, 4, 0, 0};
volatile int32_t g_hall_vel[3];

void halls_pioa_irq(void) __attribute__ ((section (".ramfunc")));
void halls_piob_irq(void) __attribute__ ((section (".ramfunc")));

void halls_init()
{
  PIO_Configure(&pin_m0_hall0, 1);
  PIO_Configure(&pin_m0_hall1, 1);
  PIO_Configure(&pin_m0_hall2, 1);

  PIO_Configure(&pin_m1_hall0, 1);
  PIO_Configure(&pin_m1_hall1, 1);
  PIO_Configure(&pin_m1_hall2, 1);

  PIO_Configure(&pin_m2_hall0, 1);
  PIO_Configure(&pin_m2_hall1, 1);
  PIO_Configure(&pin_m2_hall2, 1);

  g_hall_count_0 = g_hall_count_1 = g_hall_count_2 = 0;
  g_hall_0 = g_hall_1 = g_hall_2 = 0;
  g_hall_prev[0] = g_hall_prev[1] = g_hall_prev[2] = 0;
  g_hall_state_0 = g_hall_state_1 = g_hall_state_2 = -1;
  g_hall_vel[0] = g_hall_vel[1] = g_hall_vel[2] = 0;

  NVIC_DisableIRQ(PIOA_IRQn);
  NVIC_DisableIRQ(PIOB_IRQn);
  NVIC_ClearPendingIRQ(PIOA_IRQn);
  NVIC_ClearPendingIRQ(PIOB_IRQn);
  PIOA->PIO_ISR;
  PIOA->PIO_IDR = 0xFFFFFFFF;
  PIOA->PIO_IER = 0x00910000;  // bits 16, 20, 23 plz
  PIOB->PIO_ISR;
  PIOB->PIO_IDR = 0xFFFFFFFF; 
  PIOB->PIO_IER = 0x00006c06; // bits 1, 2, 10, 11, 13, 14 plz
  NVIC_SetPriority(PIOA_IRQn, 0); // top priority
  NVIC_SetPriority(PIOB_IRQn, 0); // top priority
  NVIC_EnableIRQ(PIOA_IRQn);
  NVIC_EnableIRQ(PIOB_IRQn);

  PMC_EnablePeripheral(ID_TC0);
  TC0->TC_BMR = 0; 
  TC0->TC_CHANNEL[0].TC_IDR = 0xff;
  TC0->TC_CHANNEL[1].TC_IDR = 0xff;
  TC0->TC_CHANNEL[2].TC_IDR = 0xff;
  TC0->TC_CHANNEL[0].TC_CCR = 0x1;
  TC0->TC_CHANNEL[0].TC_CMR = 0; // just count plz
}

void halls_pioa_irq()
{
  PIOA->PIO_ISR; // read interrupt flag to clear it
  uint32_t halls_a = REG_PIOA_PDSR;
  int next_state;
  g_hall_0 = ((halls_a & 0x00010000) >> 16) |
             ((halls_a & 0x00800000) >> 22) |
             ((halls_a & 0x00100000) >> 18);
  if (g_hall_state_0 == -1) // initialization
  {
    g_hall_state_0 = g_hall_0; 
    return;
  }
  next_state = g_hall_decode[g_hall_0 & 0x7];
  if (next_state == g_hall_forward[g_hall_state_0])
    g_hall_count_0++;
  else if (next_state == g_hall_backward[g_hall_state_0])
    g_hall_count_0--;
  g_hall_state_0 = next_state;
}

void halls_piob_irq()
{
  TC0->TC_CHANNEL[0].TC_CCR = 0x5; // reset timer
  PIOB->PIO_ISR; // read interrupt flag to clear it
  uint32_t halls_b = REG_PIOB_PDSR;
  int next_state;

  g_hall_1 = ((halls_b & 0x00002000) >> 13) |
             ((halls_b & 0x00000400) >>  9) |
             ((halls_b & 0x00004000) >> 12);
  g_hall_2 = ((halls_b & 0x00000006) >>  1) |
             ((halls_b & 0x00000800) >>  9);

  if (g_hall_state_1 == -1) // initial condition
  {
    g_hall_state_1 = g_hall_prev[1] = g_hall_1;
    g_hall_state_2 = g_hall_prev[2] = g_hall_2;
  }
  if (g_hall_1 != g_hall_prev[1])
  {
    next_state = g_hall_decode[g_hall_1];
    if (next_state == g_hall_forward[g_hall_state_1])
      g_hall_count_1++;
    else if (next_state == g_hall_backward[g_hall_state_1])
      g_hall_count_1--;
    g_hall_state_1 = next_state;
    g_hall_prev[1] = g_hall_1;
  }
  if (g_hall_2 != g_hall_prev[2])
  {
    next_state = g_hall_decode[g_hall_2];
    if (next_state == g_hall_forward[g_hall_state_2])
      g_hall_count_2++;
    else if (next_state == g_hall_backward[g_hall_state_2])
      g_hall_count_2--;
    g_hall_state_2 = next_state;
    g_hall_prev[2] = g_hall_2;
  }
  
  g_hall_vel[1] = TC0->TC_CHANNEL[0].TC_CV;
}

