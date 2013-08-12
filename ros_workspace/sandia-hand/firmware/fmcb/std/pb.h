#ifndef PB_H
#define PB_H

#include <stdint.h>
#include <stdbool.h>

void pb_init();
void pb_usart1_irq();
void pb_send_block(uint8_t *block, uint32_t len);
void pb_wait_for_traffic(uint16_t max_ms,
                         volatile uint16_t *num_bytes_recv, 
                         volatile uint8_t *bytes_recv);
void pb_systick();
void pb_idle();
void pb_set_power(bool on);
void pb_set_auto_polling(bool on);
void pb_tc0_irq();

#endif

