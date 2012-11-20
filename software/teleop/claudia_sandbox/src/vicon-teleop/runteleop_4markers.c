// file: runteleop_4markers.c
// pod: vicon-teleop

#include <stdio.h>
#include <GL/gl.h>

#include <lcm/lcm.h>
#include <inttypes.h>
#include <lcmtypes/vicon_drc.h>
#include <lcmtypes/teleop.h>

#include <bot_lcmgl_client/lcmgl.h>


// The different types of data to output
typedef enum
{
    POSITION,
    ANGLE,
    NUM_DATA_TYPES
} datatype;

static lcm_t* lcm4;
bot_lcmgl_t* lcmgl_;



void send_gl(const viconstructs_vicon_t *msg, teleop_fourmarkers_t *msgb){
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_point_size(lcmgl_, 1.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1); // Blue
  int i;
  for (i=0; i < msg->models[0].nummarkers; ++i) {
    double xyz[3];
    xyz[0] = msg->models[0].markers[i].xyz.x/1000.0;
    xyz[1] = msg->models[0].markers[i].xyz.y/1000.0;
    xyz[2] = msg->models[0].markers[i].xyz.z/1000.0;
    //cout << xyz[0] <<"|"<< xyz[1] <<"|"<< xyz[2] <<"\n"; 
    bot_lcmgl_vertex3f(lcmgl_, xyz[0], xyz[1], xyz[2]);
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);

  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_point_size(lcmgl_, 3.0f);
  bot_lcmgl_begin(lcmgl_, GL_LINES);  // render as points
  bot_lcmgl_color3f(lcmgl_, 1, 0, 0); // Reg
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_RightShoulder_a[0]/1000.0 , msgb->Marker_RightShoulder_a[1]/100.0, msgb->Marker_RightShoulder_a[2] /1000.0);
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_RightHand_a[0]/1000.0 , msgb->Marker_RightHand_a[1]/1000.0, msgb->Marker_RightHand_a[2]/1000.0 );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_RightShoulder_a[0]/1000.0 , msgb->Marker_RightShoulder_a[1]/100.0, msgb->Marker_RightShoulder_a[2] /1000.0);
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_LeftShoulder_a[0]/1000.0 , msgb->Marker_LeftShoulder_a[1]/1000.0, msgb->Marker_LeftShoulder_a[2] /1000.0);
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_LeftHand_a[0] /1000.0, msgb->Marker_LeftHand_a[1]/1000.0, msgb->Marker_LeftHand_a[2]/1000.0 );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_LeftShoulder_a[0]/1000.0 , 	msgb->Marker_LeftShoulder_a[1]/1000.0, msgb->Marker_LeftShoulder_a[2] /1000.0);
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);

  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 1, 0); // Green
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_RightShoulder_a[0]/1000.0 , msgb->Marker_RightShoulder_a[1]/100.0, msgb->Marker_RightShoulder_a[2] /1000.0);
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_RightHand_a[0]/1000.0 , msgb->Marker_RightHand_a[1]/1000.0, msgb->Marker_RightHand_a[2]/1000.0 );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_LeftHand_a[0] /1000.0, msgb->Marker_LeftHand_a[1]/1000.0, msgb->Marker_LeftHand_a[2]/1000.0 );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Marker_LeftShoulder_a[0]/1000.0 , 	msgb->Marker_LeftShoulder_a[1]/1000.0, msgb->Marker_LeftShoulder_a[2] /1000.0);
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);

  bot_lcmgl_switch_buffer(lcmgl_);  
}





void getInfo(double dest[4], viconstructs_marker_t* marker, double origin[3])
{

	dest[0] = marker->xyz.x;
	dest[1] = marker->xyz.y;
	dest[2] = marker->xyz.z;
	dest[3] = marker->o;

	dest[0] = dest[0] - origin[0];
	dest[1] = dest[1] - origin[1];
	dest[2] = dest[2] - origin[2];

}

static void get_publish_4markers (const lcm_recv_buf_t *rbuf, const char* channel, const viconstructs_vicon_t* vicon, void* user)
{
    datatype dtype = *((datatype*)user);
    teleop_fourmarkers_t vicondata = 
    {
        .timestamp = 0
    };






    int i, j;
    double waist[3];

    for(i = 0; i < vicon->nummodels; i++)
    {
    	viconstructs_model_t* model = vicon->models+i;

        //All the following segment names must coincide with the name in the model being published by the vicon SDK
        if (!strcmp (model->name, "DRC_Master_Subject_UpperBody"))
        {

        	printf("Teleoperator vicon-model have been detected");

            for(j = 0; j < model->nummarkers; j++)
            {
            	viconstructs_marker_t* marker = model->markers+j;
                if (!strcmp (marker->name, "Marker_CenterWirst_2"))
                {
                	waist[0] = marker->xyz.x;
                	waist[1] = marker->xyz.y;
                	waist[2] = marker->xyz.z;
                	printf("waist\n");
                	break;
                }
            }
            //todo: I also need to check if the origin (currently using waist) it's being seen by vicon
            //todo: some kind of redundancy (segments?)
            //todo: give error if the marker is not found in the model


            for(j = 0; j < model->nummarkers; j++)
            {
                viconstructs_marker_t* marker = model->markers+j;
                if (!strcmp (marker->name, "Marker_RightShoulder"))
                {
                    getInfo(vicondata.Marker_RightShoulder_a, marker, waist);
                    //vicondata.Marker_RightShoulder_a[0] = vicondata.Marker_RightShoulder_a[0] - waist[0];
                }    
                else if (!strcmp (marker->name, "Marker_RightHand"))
                {
                    getInfo(vicondata.Marker_RightHand_a, marker, waist);
                }        
                else if (!strcmp (marker->name, "Marker_LeftShoulder"))
                {
                    getInfo(vicondata.Marker_LeftShoulder_a, marker, waist);
                }
                else if (!strcmp (marker->name, "Marker_LeftHand"))
                {
                	getInfo(vicondata.Marker_LeftHand_a, marker, waist);
                }
                //else
                //{
                //    printf("You have an unrecognized marker name. Check this please");
                //}
            }

             
            send_gl(vicon,&vicondata);
            teleop_fourmarkers_t_publish(lcm4, "vicon4markers", &vicondata);

        }
    }

}


int main(int argc, char ** argv)
{
	// lcm4 for publishing the markers positions
    lcm4 = lcm_create(NULL);

    if (!lcm4)
    {
        return 1;
    }

    lcmgl_ = bot_lcmgl_init(lcm4, "teleop_4markers");



    lcm_t* lcm_v = lcm_create(NULL);

    if (!lcm_v)
    {
        return 1;
    }

    datatype dtype;
    dtype = POSITION;

    printf("Listening on channel --drc_vicon-- ");
    viconstructs_vicon_t_subscribe(lcm_v,"drc_vicon",get_publish_4markers,&dtype);

    for(;;)
    {
        printf("\n\nMAIN reading lcm vicon\n\n");
        lcm_handle(lcm_v);
    }

    lcm_destroy(lcm4);
    return 0;
} // end main

