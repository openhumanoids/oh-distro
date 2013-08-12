#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init();
void adc_systick();
void adc_idle();
extern volatile uint16_t g_adc_data[3];
void adc_irq();

#endif

