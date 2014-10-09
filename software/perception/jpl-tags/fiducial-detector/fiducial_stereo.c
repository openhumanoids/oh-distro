/*
 * Copyright 2012, by the California Institute of Technology. ALL
 * RIGHTS RESERVED. United States Government Sponsorship
 * acknowledged. Any commercial use must be negotiated with the Office
 * of Technology Transfer at the California Institute of Technology.
 *
 * This software may be subject to U.S. export control laws. By
 * accepting this software, the user agrees to comply with all
 * applicable U.S. export laws and regulations. User has the
 * responsibility to obtain export licenses, or other export authority
 * as may be required before exporting such information to foreign
 * countries or providing access to foreign persons.
 */

/**
@file  fiducial_stereo.c
@brief stereo msl fiducial detectors

@date  11/27/2012

@author Paul Hebert (paul.hebert@jpl.nasa.gov)
Mobility and Manipulation Group (3475), JPL
*/


#include <sys/time.h>
#include "fiducial_stereo.h"

#ifdef _OPENMP
#include <omp.h>
#endif

fiducial_stereo_t* fiducial_stereo_alloc()
{
  fiducial_stereo_t* self;

  self = (fiducial_stereo_t*) calloc(1,sizeof(fiducial_stereo_t));
  assert(self);

  int i;
  for(i = 0; i < FIDUCIAL_STEREO_NUM_CAMERAS; i++)
  {
    self->fiducial_detector[i] = fiducial_detector_alloc();
    assert(self->fiducial_detector[i]);
  } 
   
  return self;
}

void fiducial_stereo_free(fiducial_stereo_t* self)
{
  int i;
  for(i = 0; i < FIDUCIAL_STEREO_NUM_CAMERAS; i++)
  {
    if(self->fiducial_detector[i])
      fiducial_detector_free(self->fiducial_detector[i]);
  }

  if(self)
    free(self);

  return;
}

void fiducial_stereo_init(fiducial_stereo_t* self)
{
  int i;
  for(i = 0; i < FIDUCIAL_STEREO_NUM_CAMERAS; i++)
  {
    fiducial_detector_init(self->fiducial_detector[i]);
  }

  self->initial_fiducial_pose = fiducial_pose_ident();
  self->fiducial_pose = fiducial_pose_ident();

 return;
}

fiducial_detector_error_t fiducial_stereo_set_camera_models(fiducial_stereo_t* self, fiducial_stereo_cam_model_t* left_cam, fiducial_stereo_cam_model_t* right_cam )
{

  if( fiducial_detector_set_camera_models(self->fiducial_detector[FIDUCIAL_STEREO_LEFT], left_cam) != FIDUCIAL_DETECTOR_OK )
    return FIDUCIAL_DETECTOR_ERR;

  if( fiducial_detector_set_camera_models(self->fiducial_detector[FIDUCIAL_STEREO_RIGHT], right_cam) != FIDUCIAL_DETECTOR_OK )
    return FIDUCIAL_DETECTOR_ERR;

  return FIDUCIAL_DETECTOR_OK;
}

fiducial_detector_error_t fiducial_stereo_get_fiducial_pose(fiducial_stereo_t* self,  fiducial_pose_t* fd_pose)
{
  fiducial_detector_t* left  = self->fiducial_detector[FIDUCIAL_STEREO_LEFT];
  fiducial_detector_t* right = self->fiducial_detector[FIDUCIAL_STEREO_RIGHT];
  
  if(!left->fiducial_projected || !right->fiducial_projected)
    return FIDUCIAL_DETECTOR_ERR;

  // Set final pose intially to initial pose
  self->fiducial_pose = self->initial_fiducial_pose;
  left->fiducial_pose = left->initial_fiducial_pose;
  right->fiducial_pose = right->initial_fiducial_pose;

  double disparity;
  disparity = left->fiducial_location.x - right->fiducial_location.x; 

  fiducial_vec_t pw;
  double focal_length;
  double baseline;
  focal_length = left->camera.focal_length_x;
  baseline = right->camera.transform[0][3];
  
//  fprintf(stderr,"baseline: %f focal_length: %f disparity: %f image center row: %f col: %f image location row: %f col: %f\n", baseline, focal_length, disparity, left->camera.image_center_y, left->camera.image_center_x, left->fiducial_location.y, left->fiducial_location.x);
  pw.z = focal_length * baseline / disparity;
  pw.y = (-left->camera.image_center_y + left->fiducial_location.y) * pw.z / focal_length;
  pw.x = (-left->camera.image_center_x + left->fiducial_location.x) * pw.z / focal_length;
  
  pw = fiducial_vec_transform(fiducial_pose_from_transform(left->camera.transform), pw);
 // fprintf(stderr,"world pt - x: %f y: %f z: %f\n", pw.x, pw.y, pw.z);
  // Check if the found location is valid ( must be within 10cm )
  if(fiducial_vec_mag(fiducial_vec_sub(pw, left->initial_fiducial_pose.pos)) < left->params.dist_thresh)
  {
    self->fiducial_pose.pos = pw;
    left->fiducial_pose.pos = pw;
    right->fiducial_pose.pos = pw;

    fd_pose->pos.x = self->fiducial_pose.pos.x;
    fd_pose->pos.y = self->fiducial_pose.pos.y;
    fd_pose->pos.z = self->fiducial_pose.pos.z;
    fd_pose->rot.u = self->fiducial_pose.rot.u;
    fd_pose->rot.x = self->fiducial_pose.rot.x;
    fd_pose->rot.y = self->fiducial_pose.rot.y;
    fd_pose->rot.z = self->fiducial_pose.rot.z;

    return FIDUCIAL_DETECTOR_OK;
  }


  fd_pose->pos.x = self->fiducial_pose.pos.x;
  fd_pose->pos.y = self->fiducial_pose.pos.y;
  fd_pose->pos.z = self->fiducial_pose.pos.z;
  fd_pose->rot.u = self->fiducial_pose.rot.u;
  fd_pose->rot.x = self->fiducial_pose.rot.x;
  fd_pose->rot.y = self->fiducial_pose.rot.y;
  fd_pose->rot.z = self->fiducial_pose.rot.z;
  
  //fprintf(stderr,"outside limit\n");

  return FIDUCIAL_DETECTOR_ERR;
}

fiducial_detector_error_t fiducial_stereo_draw_fiducials(fiducial_stereo_t* self,
                                                         uint8_t* left_image_data, 
                                                         uint8_t* right_image_data, 
                                                         int image_cols, int image_rows, int image_channels)
{
  int c = 0;

  // Setup image data;
  uint8_t* image_data[FIDUCIAL_STEREO_NUM_CAMERAS] = {NULL};
  image_data[FIDUCIAL_STEREO_LEFT] = left_image_data;
  image_data[FIDUCIAL_STEREO_RIGHT] = right_image_data;

  // Setup error;
  fiducial_detector_error_t err[FIDUCIAL_STEREO_NUM_CAMERAS];

  // Setup image information
  int cols[FIDUCIAL_STEREO_NUM_CAMERAS];
  cols[FIDUCIAL_STEREO_LEFT] = image_cols;
  cols[FIDUCIAL_STEREO_RIGHT] = image_cols;
  int rows[FIDUCIAL_STEREO_NUM_CAMERAS];
  rows[FIDUCIAL_STEREO_LEFT] = image_rows;
  rows[FIDUCIAL_STEREO_RIGHT] = image_rows;
  int channels[FIDUCIAL_STEREO_NUM_CAMERAS];
  channels[FIDUCIAL_STEREO_LEFT] = image_channels;
  channels[FIDUCIAL_STEREO_RIGHT] = image_channels;

  // Setup num threads
  omp_set_num_threads(2);
#ifdef _OPENMP
#pragma omp parallel for 
#endif  
  for(c = 0; c < FIDUCIAL_STEREO_NUM_CAMERAS; c++)
  {
    err[c] = fiducial_detector_draw_fiducial(self->fiducial_detector[c],
                                             image_data[c],
                                             cols[c], rows[c], channels[c]);
  }

  // Check errors 
  for(c = 0; c < FIDUCIAL_STEREO_NUM_CAMERAS; c++)
  {
    if( err[c] != FIDUCIAL_DETECTOR_OK )
    {
      //fprintf(stderr," Error in drawing");
      return FIDUCIAL_DETECTOR_ERR; 
    }
  }
 

  return FIDUCIAL_DETECTOR_OK;
}
               

fiducial_detector_error_t fiducial_stereo_process(fiducial_stereo_t* self, 
                                                  uint8_t* left_image_data, 
                                                  uint8_t* right_image_data, 
                                                  int image_cols, int image_rows, int image_channels,
                                                  fiducial_pose_t initial_pose,
                                                  fiducial_pose_t *found_pose,
                                                  float *left_score,
                                                  float *right_score,
                                                  bool full_pose)
{

  int c = 0;
  float score[FIDUCIAL_STEREO_NUM_CAMERAS] = {0.0};

  // Setup initial pose;
  self->initial_fiducial_pose = initial_pose;
  fiducial_pose_t initial_fd_pose[FIDUCIAL_STEREO_NUM_CAMERAS];
  initial_fd_pose[FIDUCIAL_STEREO_LEFT] = initial_pose;
  initial_fd_pose[FIDUCIAL_STEREO_RIGHT] = initial_pose;

  // Setup image data;
  uint8_t* image_data[FIDUCIAL_STEREO_NUM_CAMERAS] = {NULL};
  image_data[FIDUCIAL_STEREO_LEFT] = left_image_data;
  image_data[FIDUCIAL_STEREO_RIGHT] = right_image_data;

  // Setup error;
  fiducial_detector_error_t err[FIDUCIAL_STEREO_NUM_CAMERAS];

  // Setup image information
  int cols[FIDUCIAL_STEREO_NUM_CAMERAS];
  cols[FIDUCIAL_STEREO_LEFT] = image_cols;
  cols[FIDUCIAL_STEREO_RIGHT] = image_cols;
  int rows[FIDUCIAL_STEREO_NUM_CAMERAS];
  rows[FIDUCIAL_STEREO_LEFT] = image_rows;
  rows[FIDUCIAL_STEREO_RIGHT] = image_rows;
  int channels[FIDUCIAL_STEREO_NUM_CAMERAS];
  channels[FIDUCIAL_STEREO_LEFT] = image_channels;
  channels[FIDUCIAL_STEREO_RIGHT] = image_channels;


#ifdef _OPENMP
  // Setup num threads
  omp_set_num_threads(2);
#endif
#ifdef _OPENMP
#pragma omp parallel for 
#endif 
  for(c = 0; c < FIDUCIAL_STEREO_NUM_CAMERAS; c++)
  {
    // Match in left and right images
    err[c] = fiducial_detector_match(self->fiducial_detector[c], image_data[c], cols[c], rows[c], channels[c], initial_fd_pose[c], &score[c]);
  }

  *left_score = score[FIDUCIAL_STEREO_LEFT];
  *right_score = score[FIDUCIAL_STEREO_RIGHT];

  // Check errors 
  for(c = 0; c < FIDUCIAL_STEREO_NUM_CAMERAS; c++)
  {
    if( err[c] != FIDUCIAL_DETECTOR_OK )
    {
      return FIDUCIAL_DETECTOR_ERR; 
    }
  }
  
  // Get the fiducial pose based on LEFT and RIGHT images (position only - x,y,z)
  if( fiducial_stereo_get_fiducial_pose(self,  found_pose) != FIDUCIAL_DETECTOR_OK )
    return FIDUCIAL_DETECTOR_ERR;

  // Get a refined fiducial 6DOF pose via gradient descent (using left image)
  if(full_pose)
  {
    if( fiducial_detector_gradient_descent(self->fiducial_detector[0], image_data[0], cols[0], rows[0], channels[0], 100, 0.001, 0.0025, found_pose, left_score) != FIDUCIAL_DETECTOR_OK )
      return FIDUCIAL_DETECTOR_ERR;
    // Set the right detector pose to the refined one
    self->fiducial_detector[FIDUCIAL_STEREO_RIGHT]->fiducial_pose = *found_pose; 
  }
  
  // Set the fiducial pose
  self->fiducial_pose = *found_pose;

  return FIDUCIAL_DETECTOR_OK;
}
