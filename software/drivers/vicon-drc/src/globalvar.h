#ifndef GLOBALVAR_H_
#define GLOBALVAR_H_

#include <inttypes.h>
#include <lcm/lcm.h>
#include "lcmtypes/vicon.h"

extern pthread_t vicon_thread;

extern int vicon_flag;
extern int vicon_init_flag;
extern uint64_t packetCount;

//vicon variables
extern unsigned short servPort;
extern char *servIP;

int global_init();

#endif /* GLOBALVAR_H_ */
