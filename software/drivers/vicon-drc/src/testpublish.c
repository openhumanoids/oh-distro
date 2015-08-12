#include <lcm/lcm.h>
#include "lcmtypes/vicon_drc.h"

static const int32_t maxmodels = 8;
static const int32_t maxmarkers = 20;
static const int32_t maxsegments = 10;
static const int32_t maxslen = 20;

/**
  * Returns a random string with a max len of maxslen
  * @return random string
  */
char* randstr()
{
    int32_t slen = (rand() % maxslen) + 1;
    char* out = malloc((slen + 1) * sizeof(char));
    int32_t i;
    for(i = 0; i < slen; i ++)
    {
        out[i] = 97+(rand()%26);
    }
    out[slen] = '\0';
    return out;
}

/**
  * Generate random XYZ tuple
  * @return random XYZ tuple
  */
viconstructs_xyz_t randxyz()
{
    viconstructs_xyz_t out = {rand(), rand(), rand()};
    return out;
}

/**
  * Publishes a random number of models, each with a random number of markers
  * and segments, everything with random names and data values
  */
int main()
{
    srand(time(NULL));
    //lcm_t* lcm = lcm_create(NULL);
    lcm_t* lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

    if (!lcm)
        return 1;

    viconstructs_vicon_t* vicon = malloc(sizeof(viconstructs_vicon_t));
    vicon->nummodels = (rand() % maxmodels) + 1;
    vicon->models = malloc(vicon->nummodels*sizeof(viconstructs_model_t));
    int32_t i, j;
    for(i = 0; i < vicon->nummodels; i++)
    {
        viconstructs_model_t* model = vicon->models + i;
        model->name = randstr();
        model->nummarkers = (rand() % maxmarkers) + 1;
        model->numsegments = (rand() % maxsegments) + 1;
        model->markers = malloc(model->nummarkers * sizeof(viconstructs_marker_t));
        model->segments = malloc(model->numsegments * sizeof(viconstructs_segment_t));

        for(j = 0; j < model->nummarkers; j++)
        {
            viconstructs_marker_t* marker = model->markers + j;
            marker->name = randstr();
            marker->xyz = randxyz();
            marker->o = rand();
        }
        
        for(j = 0; j < model->numsegments; j++)
        {
            viconstructs_segment_t* segment = model->segments + j;
            segment->name = randstr();
            segment->A = randxyz();
            segment->T = randxyz();
            segment->ba = randxyz();
            segment->bt = randxyz();
            segment->r = randxyz();
            segment->t = randxyz();
        }
    }

    viconstructs_vicon_t_publish(lcm, "drc_vicon", vicon);
    viconstructs_vicon_t_destroy(vicon);

    lcm_destroy(lcm);
    return 0;
}
