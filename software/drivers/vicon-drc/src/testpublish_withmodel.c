#include <lcm/lcm.h>
#include "lcmtypes/vicon_drc.h"

static const int32_t maxmodels = 8;
static const int32_t maxmarkers = 20;
static const int32_t maxsegments = 10;
static const int32_t maxslen = 20;

static const double maxrand = 400.0;

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

// Random floating point value on [0,1]
double randf()
{
    return 1.0*rand()/RAND_MAX;
}

// Random floating point value on [-maxrand,maxrand]
double randval()
{
    return (2.0*randf()-1.0)*maxrand;
}

/**
  * Generate random XYZ tuple
  * @return random XYZ tuple
  */
viconstructs_xyz_t randxyz()
{
    viconstructs_xyz_t out = {randval(), randval(), randval()};
    return out;
}

/**
  * Publishes a model with four segments complying with the names and data
  * values expected by the rest of the vicon->sim stack
  */
int main()
{
    srand(time(NULL));
    //lcm_t* lcm = lcm_create(NULL);
    lcm_t* lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

    if (!lcm)
        return 1;

    viconstructs_vicon_t* vicon = malloc(sizeof(viconstructs_vicon_t));

    vicon->nummodels = 1;
    vicon->models = malloc(vicon->nummodels*sizeof(viconstructs_model_t));
    viconstructs_model_t* model = vicon->models;

    char *modelname = "Eric";
    char *LH = "lhand";
    char *LS = "lshoulder";
    char *RH = "rhand";
    char *RS = "rshoulder";

    model->name = modelname;
    model->numsegments = 4;
    model->nummarkers = 0;

    model->segments = malloc(model->numsegments * sizeof(viconstructs_segment_t));
    model->markers = malloc(sizeof(viconstructs_marker_t));

    model->segments[0].name = LH;
    model->segments[1].name = LS;
    model->segments[2].name = RH;
    model->segments[3].name = RS;

    int j;

    for(j = 0; j < model->numsegments; j++)
    {
        viconstructs_segment_t* segment = model->segments + j;
   
        segment->A = randxyz();
        segment->T = randxyz();
        segment->ba = randxyz();
        segment->bt = randxyz();
        segment->r = randxyz();
        segment->t = randxyz();
    }

    viconstructs_vicon_t_publish(lcm, "drc_vicon", vicon);
    // This line is sad with a marker array length of 0 :(
    //viconstructs_vicon_t_destroy(vicon);

    lcm_destroy(lcm);
    return 0;
}
