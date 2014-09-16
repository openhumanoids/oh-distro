#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui_c.h>

#include "fiducial_stereo.h"

#ifdef _OPENMP
#include <omp.h>
#endif


// defined below after main
int load_camera_model(const char* filename, fiducial_stereo_cam_model_t* stereo_cam_model);

int main(int argc, char** argv)
{
  fiducial_stereo_cam_model_t camera[FIDUCIAL_STEREO_NUM_CAMERAS];
  int cols, rows, channels;
  int c;
  
  // --------------------------------
  // load the left camera model
  // --------------------------------
  if(load_camera_model("left_rect.cam", &camera[FIDUCIAL_STEREO_LEFT])!=0)
  {
    fprintf(stderr, "failed to load camera model! \n");
    return -1;
  }

  // --------------------------------
  // load the right camera model
  // --------------------------------
  if(load_camera_model("right_rect.cam", &camera[FIDUCIAL_STEREO_RIGHT])!=0)
  {
    fprintf(stderr, "failed to load camera model! \n");
    return -1;
  }

  
  // -----------------------------------
  // load the camera images
  // -----------------------------------
    IplImage *img[FIDUCIAL_STEREO_NUM_CAMERAS] = {NULL};
    IplImage *debug_img[FIDUCIAL_STEREO_NUM_CAMERAS] = {NULL};
    img[FIDUCIAL_STEREO_LEFT] = cvLoadImage("left.pnm", CV_LOAD_IMAGE_COLOR);
    debug_img[FIDUCIAL_STEREO_LEFT] = cvCreateImage(cvSize(img[FIDUCIAL_STEREO_LEFT]->width, img[FIDUCIAL_STEREO_LEFT]->height), IPL_DEPTH_8U, img[FIDUCIAL_STEREO_LEFT]->nChannels);
    cvCopy(img[FIDUCIAL_STEREO_LEFT], debug_img[FIDUCIAL_STEREO_LEFT], 0);
 
    img[FIDUCIAL_STEREO_RIGHT] = cvLoadImage("right.pnm", CV_LOAD_IMAGE_COLOR);
    debug_img[FIDUCIAL_STEREO_RIGHT] = cvCreateImage(cvSize(img[FIDUCIAL_STEREO_RIGHT]->width, img[FIDUCIAL_STEREO_RIGHT]->height), IPL_DEPTH_8U, img[FIDUCIAL_STEREO_RIGHT]->nChannels);
    cvCopy(img[FIDUCIAL_STEREO_RIGHT], debug_img[FIDUCIAL_STEREO_RIGHT], 0);
  
  cols = img[FIDUCIAL_STEREO_LEFT]->width;
  rows = img[FIDUCIAL_STEREO_LEFT]->height;
  channels = img[FIDUCIAL_STEREO_LEFT]->nChannels;



  
  // ------------------------------------
  // load the initial pose from kin
  // ------------------------------------
  int ret;
  FILE *pfile = NULL; 
  fiducial_pose_t initial_fd_pose, new_fd_pose;
  pfile = fopen("initial_location.txt", "r");
  assert(pfile);  
  ret = fscanf(pfile,
      "%lf %lf %lf %lf %lf %lf %lf",
      &initial_fd_pose.pos.x,
      &initial_fd_pose.pos.y,
      &initial_fd_pose.pos.z,
      &initial_fd_pose.rot.u,
      &initial_fd_pose.rot.x,
      &initial_fd_pose.rot.y,
      &initial_fd_pose.rot.z);
  fclose(pfile);
  if(ret!=7)
  {
    fprintf(stderr, "unable to read initial_pose.txt!\n");
    return -1;
  }

  // ------------------------------------
  // If want to deviate the initial fd pose 
  // to test convergence
  // ------------------------------------
  /* 
  double r,p,y;
  fiducial_rot_to_rpy(initial_fd_pose.rot, &r, &p, &y); 
  y = y + 20.0 * M_PI/ 180.0;
  p = p - 20.0 * M_PI/ 180.0;
  //r = r - 5.0 * M_PI/ 180.0;
  initial_fd_pose.rot = fiducial_rot_from_rpy(r,p,y);
  */ 

  // ----------------------------
  // setup the fiducial detector
  // ----------------------------
  float left_score = 0;
  float right_score = 0;

  fiducial_stereo_t* fiducial_stereo;
  fiducial_stereo = fiducial_stereo_alloc();
  fiducial_stereo_init(fiducial_stereo);
  // ----------------------------
  // setup the camera models
  // ----------------------------
  if( fiducial_stereo_set_camera_models(fiducial_stereo, &camera[FIDUCIAL_STEREO_LEFT], &camera[FIDUCIAL_STEREO_RIGHT]) != FIDUCIAL_DETECTOR_OK)
  {
    fprintf(stderr, "failed to set camera models in fiducial detector!\n");
    return -1;
  }

  int i;
 
  struct timeval stv;
  double t1, t2;
  int num_iterations = 100;
  double rate = 0.0;

 
  for(i = 0; i < num_iterations; i++)
  {
      gettimeofday(&stv, NULL);
      t1 = (double)(stv.tv_sec + stv.tv_usec/1000000.0);

      // ----------------------------------------
      // process detector and get fiducial pose
      // ----------------------------------------
      fiducial_stereo_process(fiducial_stereo, 
                              (uint8_t*) img[FIDUCIAL_STEREO_LEFT]->imageData, 
                              (uint8_t*) img[FIDUCIAL_STEREO_RIGHT]->imageData, 
                              cols, rows, channels,
                              initial_fd_pose,
                              &new_fd_pose,
                              &left_score,
                              &right_score,
                              false);

      gettimeofday(&stv, NULL);
      t2 = (double)(stv.tv_sec + stv.tv_usec/1000000.0);
    // ----------------------------------------
    // print out results
    // ----------------------------------------
    fprintf(stderr, "-------------------------------------------------------------------\n");
    fprintf(stderr, "kinematic pos: %lf %lf %lf [m]    quat: %lf %lf %lf %lf\n",
            initial_fd_pose.pos.x,
            initial_fd_pose.pos.y,
            initial_fd_pose.pos.z,
            initial_fd_pose.rot.u,
            initial_fd_pose.rot.x,
            initial_fd_pose.rot.y,
            initial_fd_pose.rot.z);
    fprintf(stderr, "-------------------------------------------------------------------\n");
    fprintf(stderr, "-------------------------------------------------------------------\n");
    fprintf(stderr, "estimated pos: %lf %lf %lf [m]    quat: %lf %lf %lf %lf    left-score: %lf     right-score: %lf\n",
            new_fd_pose.pos.x,
            new_fd_pose.pos.y,
            new_fd_pose.pos.z,
            new_fd_pose.rot.u,
            new_fd_pose.rot.x,
            new_fd_pose.rot.y,
            new_fd_pose.rot.z,
            left_score, right_score);
    fprintf(stderr, "-------------------------------------------------------------------\n");
    fprintf(stderr, "-------------------------------------------------------------------\n");
    fprintf(stderr, "dT: %lf   (%lf Hz)\n", t2 - t1, 1.0/(t2 - t1));
    fprintf(stderr, "-------------------------------------------------------------------\n");


    rate += 1.0/(t2-t1);
  }

  fprintf(stderr,"average rate: %lf Hz\n", rate/num_iterations);

  // ------------------------------------
  // draw debug images
  // ------------------------------------
  if ( fiducial_stereo_draw_fiducials(fiducial_stereo,
                                        (uint8_t*)debug_img[FIDUCIAL_STEREO_LEFT]->imageData,
                                        (uint8_t*)debug_img[FIDUCIAL_STEREO_RIGHT]->imageData,
                                        cols, rows, channels)  != FIDUCIAL_DETECTOR_OK)
  {
    fprintf(stderr, "failed to draw debug images!\n");
    return -1;
  }
  cvSaveImage("left_debug_img.ppm", debug_img[FIDUCIAL_STEREO_LEFT], 0);
  cvSaveImage("right_debug_img.ppm", debug_img[FIDUCIAL_STEREO_RIGHT], 0);

  // --------------------------------
  // cleanup
  // --------------------------------
  fiducial_stereo_free(fiducial_stereo);
  for(c = 0; c < FIDUCIAL_STEREO_NUM_CAMERAS; c++)
  {
    
    if(img[c])
      cvReleaseImage(&img[c]);
    if(debug_img[c])
      cvReleaseImage(&debug_img[c]);

  }
  
  return 0;
}



// function that loads the necessary cam models from file
int load_camera_model(const char* filename, fiducial_stereo_cam_model_t* stereo_cam_model)
{
  FILE *pfile = NULL;  
  char text[100];
  int ret, i;
  
  // --------------------------------
  // load the camera params
  // --------------------------------
  pfile = fopen(filename,"r");
  ret = fscanf(pfile, "%s %d", &text[0], &stereo_cam_model->cols);
  if(ret != 2)
  {
    fclose(pfile);
    fprintf(stderr,"malformed line in %s! Abort.\n", filename);
    return -1;
  }
  ret = fscanf(pfile, "%s %d", &text[0], &stereo_cam_model->rows);
  if(ret != 2)
  {
    fclose(pfile);
    fprintf(stderr,"malformed line in %s! Abort.\n", filename);
    return -1;
  }
  ret = fscanf(pfile, "%s %lf", &text[0], &stereo_cam_model->focal_length_x);
  if(ret != 2)
  {
    fclose(pfile);
    fprintf(stderr,"malformed line in %s! Abort.\n", filename);
    return -1;
  }
  ret = fscanf(pfile, "%s %lf", &text[0], &stereo_cam_model->focal_length_y);
  if(ret != 2)
  {
    fclose(pfile);
    fprintf(stderr,"malformed line in %s! Abort.\n", filename);
    return -1;
  }
  ret = fscanf(pfile, "%s %lf", &text[0], &stereo_cam_model->image_center_x);
  if(ret != 2)
  {
    fclose(pfile);
    fprintf(stderr,"malformed line in %s! Abort.\n", filename);
    return -1;
  }
  ret = fscanf(pfile, "%s %lf", &text[0], &stereo_cam_model->image_center_y);
  if(ret != 2)
  {
    fclose(pfile);
    fprintf(stderr,"malformed line in %s! Abort.\n", filename);
    return -1;
  }
  ret = fscanf(pfile, "%s %lf %lf %lf %lf", &text[0],
               &stereo_cam_model->transform[0][0], &stereo_cam_model->transform[0][1], &stereo_cam_model->transform[0][2], &stereo_cam_model->transform[0][3]);
  if(ret != 5)
  {
    fclose(pfile);
    fprintf(stderr,"malformed line in %s! Abort.\n", filename);
    return -1;
  }
  for(i=1; i<4; i++)
  {
    ret = fscanf(pfile, "%lf %lf %lf %lf",
                 &stereo_cam_model->transform[i][0], &stereo_cam_model->transform[i][1], &stereo_cam_model->transform[i][2], &stereo_cam_model->transform[i][3]);
    if(ret != 4)
    {
      fclose(pfile);
      fprintf(stderr,"malformed line in %s! Abort.\n", filename);
      return -1;
    }
  }
  fclose(pfile);

  return 0;
}
