#ifndef COMMS_H
#define COMMS_H

void comms_init();
void comms_irq();
void comms_systick();
void comms_idle();
#define COMMS_MAX_PACKET_LENGTH 1024

#endif
