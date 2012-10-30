// file: listener.c
//
// LCM example program.
//
// compile with:
//  $ gcc -o listener listener.c -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o listener listener.c `pkg-config --cflags --libs lcm`

#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include "exlcm_example_t.h"

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

static void
my_handler(const lcm_recv_buf_t *rbuf, const char * channel,
        const exlcm_example_t * msg, void * user)
{
    int i;
    printf("Received message on channel \"%s\":\n", channel);
    printf("  timestamp   = %"PRId64"\n", msg->timestamp);
    printf("  position    = (%f, %f, %f)\n",
            msg->position[0], msg->position[1], msg->position[2]);
    printf("  orientation = (%f, %f, %f, %f)\n",
            msg->orientation[0], msg->orientation[1], msg->orientation[2],
            msg->orientation[3]);
    printf("  ranges[%d]:",msg->num_ranges);
    for(i = 0; i < MIN(msg->num_ranges,15); i++) //print first 15
        printf(" %d", msg->ranges[i]);
    if(msg->num_ranges>15) printf("...");
    printf("\n");
    printf("  name        = '%s'\n", msg->name);
    printf("  enabled     = %d\n", msg->enabled);
}

int
main(int argc, char ** argv)
{
    lcm_t * lcm = lcm_create(NULL);
    if(!lcm)
        return 1;

    exlcm_example_t_subscribe(lcm, "EXAMPLE", &my_handler, NULL);

    while(1)
        lcm_handle(lcm);

    lcm_destroy(lcm);
    return 0;
}
