#include "adc.h"
#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"

static const int ADC_READ_INPUT_VOLTAGE    = 0;
static const int ADC_READ_TEMPERATURE      = 1;
static const int ADC_READ_PHALANGE_CURRENT = 2;

static const int ADC_CH_INPUT_VOLTAGE = 7; // PB3 = AD7
static const int ADC_CH_TEMPERATURE = 15; // fake adc input
static const int ADC_CH_PHALANGE_CURRENT = 4; // pb0 = ad4
#if 0
static int8_t g_adc_whos_next = -1;
static int8_t g_adc_whos_now = 0;
#endif
volatile uint16_t g_adc_data[3];

void adc_init()
{
  PMC_EnablePeripheral(ID_ADC);
  ADC->ADC_CR = ADC_CR_SWRST;
  ADC->ADC_MR = ADC_MR_TRANSFER(1) | 
                ADC_MR_TRACKTIM(1) |
                ADC_MR_SETTLING(3) |
                ADC_MR_PRESCAL(4)  |   // adc clock = 64 MHz / ((4+1)*2) = 6.4
                ADC_MR_FREERUN     |
                ADC_MR_STARTUP_SUT512;
  ADC->ADC_CHER = (1 << ADC_CH_INPUT_VOLTAGE)    |
                  (1 << ADC_CH_TEMPERATURE)      |
                  (1 << ADC_CH_PHALANGE_CURRENT);
  ADC->ADC_IDR = 0xffffffff;
  ADC->ADC_ACR = ADC_ACR_TSON; // turn on temperature sensor
  ADC->ADC_CR = ADC_CR_START; // start a conversion
  while (!(ADC->ADC_ISR & ADC_ISR_DRDY)) { }
  ADC->ADC_LCDR;
  ADC->ADC_CR = ADC_CR_START; // start another conversion
  while (!(ADC->ADC_ISR & ADC_ISR_DRDY)) { }
  ADC->ADC_LCDR;
  ADC->ADC_IER = ADC_IER_DRDY; // fire ADC interrupt on any conversion complete
  for (int i = 0; i < 3; i++)
    g_adc_data[i] = 0x42 + i;
  NVIC_SetPriority(ADC_IRQn, 11); // really low priority...
  NVIC_EnableIRQ(ADC_IRQn);
}

void adc_systick()
{
  return; // free-running now...
  /*
  static volatile int s_adc_systick_count = 0;
  s_adc_systick_count++;
  if (g_adc_whos_next < 0)
  {
    if (s_adc_systick_count % 100 == 0)
      g_adc_whos_next = ADC_READ_INPUT_VOLTAGE;
    else if (s_adc_systick_count % 100 == 10)
      g_adc_whos_next = ADC_READ_PHALANGE_CURRENT;
    else if (s_adc_systick_count % 100 == 20)
      g_adc_whos_next = ADC_READ_TEMPERATURE;
  }
  */
}

void adc_idle()
{
  // free-running now...
#if 0
  if (g_adc_whos_next >= 0)
  {
    /*
    ADC->ADC_CHDR = 0xffff;
    if (g_adc_whos_next == ADC_READ_TEMPERATURE)
      ADC->ADC_CHER = 1 << ADC_CH_TEMPERATURE;
    else if (g_adc_whos_next == ADC_READ_INPUT_VOLTAGE)
      ADC->ADC_CHER = 1 << ADC_CH_INPUT_VOLTAGE;
    else if (g_adc_whos_next == ADC_READ_PHALANGE_CURRENT)
      ADC->ADC_CHER = 1 << ADC_CH_PHALANGE_CURRENT;
    */
    g_adc_whos_now = g_adc_whos_next;
    g_adc_whos_next = -1; // interrupt will fire when it's done.
    ADC->ADC_CR = ADC_CR_START; // start the conversion
  }
#endif
}

void adc_irq()
{
  // copy to global buffer, maybe filter first, etc. etc.
  //g_adc_data[g_adc_whos_now] = ADC->ADC_LCDR; // read ADC_LCDR clears interrupt
  g_adc_data[0] = ADC->ADC_CDR[7];
  g_adc_data[1] = ADC->ADC_CDR[15];
  g_adc_data[2] = ADC->ADC_CDR[4];
  ADC->ADC_LCDR;
}

