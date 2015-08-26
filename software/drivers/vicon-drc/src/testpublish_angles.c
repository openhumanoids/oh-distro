#include <lcm/lcm.h>
#include <math.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "lcmtypes/vicon_drc.h"

/**
  * Receives a UDP packet of 12 doubles, places them in the angles and broadcasts the message
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

    char *modelname = "teleoperator";
    char *LH = "LeftHand";
    char *LS = "LeftShoulder";
    char *RH = "RightHand";
    char *RS = "RightShoulder";

    model->name = modelname;
    model->numsegments = 4;
    model->nummarkers = 0;

    model->segments = malloc(model->numsegments * sizeof(viconstructs_segment_t));
    model->markers = malloc(sizeof(viconstructs_marker_t));

    model->segments[0].name = LH;
    model->segments[1].name = LS;
    model->segments[2].name = RH;
    model->segments[3].name = RS;

    viconstructs_xyz_t zero = {0.0, 0.0, 0.0};
    int j;
    for(j = 0; j < model->numsegments; j++)
    {
        viconstructs_segment_t* segment = model->segments + j;
        segment->A = zero;
        segment->T = zero;
        segment->ba = zero;
        segment->bt = zero;
        segment->r = zero;
        segment->t = zero;
    }

    // UDP client stuff
    int sockfd,n;
    struct sockaddr_in servaddr;
    socklen_t len;
    char msg[sizeof(double)*12];
    //char msg[1024];
    double* vals = msg;
    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=(INADDR_ANY);
    servaddr.sin_port=htons(4321);
    bind(sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr));

    for (;;)
    {
        len = sizeof(servaddr);
        n = recvfrom(sockfd,msg,sizeof(double)*12,0,(struct sockaddr *)&servaddr,&len);
        /*
        int i;
        for(i=0;i<12;i++)
        {
            printf("%d %f\n",i,vals[i]);
        }
        printf("\n");
        */
        printf("got message\n");

        model->segments[0].A.x = vals[0];
        model->segments[0].A.y = vals[1];
        model->segments[0].A.z = vals[2];
        model->segments[1].A.x = vals[3];
        model->segments[1].A.y = vals[4];
        model->segments[1].A.z = vals[5];
        model->segments[2].A.x = vals[6];
        model->segments[2].A.y = vals[7];
        model->segments[2].A.z = vals[8];
        model->segments[3].A.x = vals[9];
        model->segments[3].A.y = vals[10];
        model->segments[3].A.z = vals[11];
        viconstructs_vicon_t_publish(lcm, "drc_vicon", vicon);
    }

    // This line is sad with a marker array length of 0 :(
    //viconstructs_vicon_t_destroy(vicon);

    lcm_destroy(lcm);
    return 0;
}
