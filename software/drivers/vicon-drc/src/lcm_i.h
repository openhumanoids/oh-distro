#ifndef LCM_I_H_
#define LCM_I_H_

#include <stdint.h>
#include "globalvar.h"

int lcm_publish_init();
void lcm_publish_vicon(viconstructs_vicon_t* vicon);
int lcm_glider_vicon_destroy();

#endif /* LCM_I_H_ */
