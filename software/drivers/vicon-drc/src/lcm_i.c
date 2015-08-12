// file: lmc_i.c
//
// lcm interface program.
//
#include <stdio.h>
#include <stdlib.h>    
#include "lcm_i.h"
#include "main.h"

lcm_t * lcm;

void lcm_publish_vicon(viconstructs_vicon_t* vicon)
{
    viconstructs_vicon_t_publish(lcm, "drc_vicon", vicon);

}

int lcm_publish_init()
{

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
        return 1;

return 0;

}

int lcm_glider_vicon_destroy()
{
	lcm_destroy(lcm);
        return 0;

}
