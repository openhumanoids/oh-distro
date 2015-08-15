#include <lcm/lcm.h>
#include <inttypes.h>
#include "lcmtypes/vicon.h"

/**
  * Accepts a viconstructs_vicon_t* struct and prints out its values
  */
static void handler(const lcm_recv_buf_t *rbuf, const char* channel, const viconstructs_vicon_t* vicon, void* user)
{
    printf("Vicon: %" PRId32 " Models\n", vicon->nummodels);
    int32_t i, j;
    for(i = 0; i < vicon->nummodels; i++)
    {
        viconstructs_model_t* model = vicon->models+i;
        printf("Model %" PRId32 ": %s\n", i+1, model->name);

        printf("%" PRId32 "  Markers:\n", model->nummarkers);
        for(j = 0; j < model->nummarkers; j++)
        {
            viconstructs_marker_t* marker = model->markers+j;
            printf("Marker: %s\n", marker->name);
            printf("x: %f y: %f z: %f o: %f\n", marker->xyz.x, marker->xyz.y, marker->xyz.z, marker->o);
            printf("\n");
        }

        printf("\n%" PRId32 " Segments:\n", model->numsegments);
        for(j = 0; j < model->numsegments; j++)
        {
            viconstructs_segment_t* segment = model->segments+j;
            printf("Segment: %s\n", segment->name);
            printf("A: x: %f y: %f z: %f\n", segment->A.x, segment->A.y, segment->A.z);
            printf("T: x: %f y: %f z: %f\n", segment->T.x, segment->T.y, segment->T.z);
            printf("ba: x: %f y: %f z: %f\n", segment->ba.x, segment->ba.y, segment->ba.z);
            printf("bt: x: %f y: %f z: %f\n", segment->bt.x, segment->bt.y, segment->bt.z);
            //printf("r: x: %f y: %f z: %f\n", segment->r.x, segment->r.y, segment->r.z);
            //printf("t: x: %f y: %f z: %f\n", segment->t.x, segment->t.y, segment->t.z);
            printf("\n");
        }
        printf("\n\n");
    }
}

/**
  * Debugger program to explore broadcast data more easily
  */
int main()
{
    lcm_t* lcm = lcm_create(NULL);
    if (!lcm)
        return 1;

    printf("Listening on channel \"drc_vicon\"");
    viconstructs_vicon_t_subscribe(lcm,"drc_vicon",handler,NULL);

    for(;;)
        lcm_handle(lcm);

    lcm_destroy(lcm);
    return 0;
}
