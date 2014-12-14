// file: runteleop_4markers.c
// pod: vicon-teleop

#include <stdio.h>
#include <GL/gl.h>
#include <iostream>

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

  //plot points for all markers in vicon world
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_point_size(lcmgl_, 3.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1); // Blue

  int i;
  for (i=0; i < msg->models[0].nummarkers; ++i) {
    double xyz[3];
    xyz[0] = msg->models[0].markers[i].xyz.x/1000.0;
    xyz[1] = msg->models[0].markers[i].xyz.y/1000.0;
    xyz[2] = msg->models[0].markers[i].xyz.z/1000.0;
//    std::cout << xyz[0] <<"|"<< xyz[1] <<"|"<< xyz[2] <<"\n";
    bot_lcmgl_vertex3f(lcmgl_, xyz[0], xyz[1], xyz[2]);
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);



  //plot points for segments in vicon world:
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_point_size(lcmgl_, 3.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 1, 0, 0); //

  for (i=0; i < msg->models[0].numsegments; ++i) {
    double xyz_segments[3];
    xyz_segments[0] = msg->models[0].segments[i].T.x/1000.0;
    xyz_segments[1] = msg->models[0].segments[i].T.y/1000.0;
    xyz_segments[2] = msg->models[0].segments[i].T.z/1000.0;
    //cout << xyz[0] <<"|"<< xyz[1] <<"|"<< xyz[2] <<"\n";
    bot_lcmgl_vertex3f(lcmgl_, xyz_segments[0], xyz_segments[1], xyz_segments[2]);
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);





  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_point_size(lcmgl_, 3.0f);
  bot_lcmgl_begin(lcmgl_, GL_LINES);  // render as lines
  bot_lcmgl_color3f(lcmgl_, 1, 0, 0); // Red
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightShoulder_c[0]/1000.0 , msgb->Segment_RightShoulder_c[1]/1000.0, msgb->Segment_RightShoulder_c[2] /1000.0);
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightHand_c[0]/1000.0 , msgb->Segment_RightHand_c[1]/1000.0, msgb->Segment_RightHand_c[2]/1000.0 );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightShoulder_c[0]/1000.0 , msgb->Segment_RightShoulder_c[1]/1000.0, msgb->Segment_RightShoulder_c[2] /1000.0);
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftShoulder_c[0]/1000.0 , msgb->Segment_LeftShoulder_c[1]/1000.0, msgb->Segment_LeftShoulder_c[2] /1000.0);
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftHand_c[0] /1000.0, msgb->Segment_LeftHand_c[1]/1000.0, msgb->Segment_LeftHand_c[2]/1000.0 );
  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftShoulder_c[0]/1000.0 , 	msgb->Segment_LeftShoulder_c[1]/1000.0, msgb->Segment_LeftShoulder_c[2] /1000.0);
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);

//  bot_lcmgl_push_matrix(lcmgl_);
//  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
//  bot_lcmgl_color3f(lcmgl_, 0, 1, 0); // Green
//  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightShoulder_a[0]/1000.0 , msgb->Segment_RightShoulder_a[1]/100.0, msgb->Segment_RightShoulder_a[2] /1000.0);
//  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightHand_a[0]/1000.0 , msgb->Segment_RightHand_a[1]/1000.0, msgb->Segment_RightHand_a[2]/1000.0 );
//  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftHand_a[0] /1000.0, msgb->Segment_LeftHand_a[1]/1000.0, msgb->Segment_LeftHand_a[2]/1000.0 );
//  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftShoulder_a[0]/1000.0 , 	msgb->Segment_LeftShoulder_a[1]/1000.0, msgb->Segment_LeftShoulder_a[2] /1000.0);
//  bot_lcmgl_end(lcmgl_);
//  bot_lcmgl_pop_matrix(lcmgl_);



  bot_lcmgl_switch_buffer(lcmgl_);

}





void getInfo(double dest[3], viconstructs_segment_t* segment, double ViconBodyOrigin[3])
{

	dest[0] = segment->T.x ;//- ViconBodyOrigin[0];
	dest[1] = segment->T.y ;//- ViconBodyOrigin[1];
	dest[2] = segment->T.z + 200; //to adjust height. I one to improve this.

//	  bot_lcmgl_push_matrix(lcmgl_);
//	  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
//	  bot_lcmgl_color3f(lcmgl_, 0, 1, 0); // Green
//	  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightShoulder_a[0]/1000.0 , msgb->Segment_RightShoulder_a[1]/100.0, msgb->Segment_RightShoulder_a[2] /1000.0);
//	  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_RightHand_a[0]/1000.0 , msgb->Segment_RightHand_a[1]/1000.0, msgb->Segment_RightHand_a[2]/1000.0 );
//	  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftHand_a[0] /1000.0, msgb->Segment_LeftHand_a[1]/1000.0, msgb->Segment_LeftHand_a[2]/1000.0 );
//	  bot_lcmgl_vertex3f(lcmgl_, msgb->Segment_LeftShoulder_a[0]/1000.0 , 	msgb->Segment_LeftShoulder_a[1]/1000.0, msgb->Segment_LeftShoulder_a[2] /1000.0);
//	  bot_lcmgl_end(lcmgl_);
//	  bot_lcmgl_pop_matrix(lcmgl_);
//	  bot_lcmgl_switch_buffer(lcmgl_);

}

static void get_publish_4markers (const lcm_recv_buf_t *rbuf, const char* channel, const viconstructs_vicon_t* vicon, void* user)
{
    datatype dtype = *((datatype*)user);
    teleop_fourmarkers_t vicondata;

    //I'll correct this timestamp
    vicondata.timestamp = 0;

    int i, j;
    double waist[3];

    for(i = 0; i < vicon->nummodels; i++)
    {
    	viconstructs_model_t* model = vicon->models+i;

        //All the following segment names must coincide with the name in the model being published by the vicon SDK
        if (!strcmp (model->name, "DRC_PALADIN_MODEL_v1"))
        {

        	printf("Teleoperator vicon-model have been detected");

//            for(j = 0; j < model->nummarkers; j++)
//            {
//            	viconstructs_marker_t* marker = model->markers+j;
//                if (!strcmp (marker->name, "Marker_CenterWaist_2"))
//                {
//                	waist[0] = marker->xyz.x;
//                	waist[1] = marker->xyz.y;
//                	waist[2] = marker->xyz.z;
//                	printf("waist\n");
//                	break;
//                }
//            }
            //todo: I also need to check if the origin (currently using waist) it's being seen by vicon
            //todo: some kind of redundancy (segments?)
            //todo: give error if the marker is not found in the model

            for(j = 0; j < model->numsegments; j++)
            {
                viconstructs_segment_t* segment = model->segments+j;
                if (!strcmp (segment->name, "Segment_Waist"))
                {
                	waist[0] = segment->T.x;
                	waist[1] = segment->T.y;
                	waist[2] = segment->T.z;
                }
//                else if (!strcmp (segment->name, "Segment_Chest"))
//                {
//                    getInfo(vicondata.right_hand, segment);
//                }
                if (!strcmp (segment->name, "Segment_RightShoulder"))
                {
                    getInfo(vicondata.Segment_RightShoulder_c, segment, waist);
                }
                else if (!strcmp (segment->name, "Segment_RightHand"))
                {
                    getInfo(vicondata.Segment_RightHand_c, segment, waist);
                }
                else if (!strcmp (segment->name, "Segment_LeftShoulder"))
                {
                    getInfo(vicondata.Segment_LeftShoulder_c, segment, waist);
                }
                else if (!strcmp (segment->name, "Segment_LeftHand"))
                {
                    getInfo(vicondata.Segment_LeftHand_c, segment, waist);
                }
                else
                {
                	continue;
                    //printf("There is an unused segment"); //that's ok, I have more segments in he model
                }
            }


//            for(j = 0; j < model->nummarkers; j++)
//            {
//                viconstructs_marker_t* marker = model->markers+j;
//                if (!strcmp (marker->name, "Marker_RightShoulder"))
//                {
//                    getInfo(vicondata.Marker_RightShoulder_a, marker, waist);
//                    //vicondata.Marker_RightShoulder_a[0] = vicondata.Marker_RightShoulder_a[0] - waist[0];
//                }
//                else if (!strcmp (marker->name, "Marker_RightHand"))
//                {
//                    getInfo(vicondata.Marker_RightHand_a, marker, waist);
//                }
//                else if (!strcmp (marker->name, "Marker_LeftShoulder"))
//                {
//                    getInfo(vicondata.Marker_LeftShoulder_a, marker, waist);
//                }
//                else if (!strcmp (marker->name, "Marker_LeftHand"))
//                {
//                	getInfo(vicondata.Marker_LeftHand_a, marker, waist);
//                }
//                //else
//                //{
//                //    printf("You have an unrecognized marker name. Check this please");
//                //}
//            }

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

