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
@file  fiducial_detector.c
@brief detector for msl fiducial

@date  08/29/2012

@author Paul Hebert (paul.hebert@jpl.nasa.gov)
Mobility and Manipulation Group (3475), JPL
*/


#include "fiducial_detector.h"

#ifdef _OPENMP
#include <omp.h>
#endif


//-----------------------------------------------------------------------------
//  THE FIDUCIAL TEMPLATE IS ORIENTED IN THE FOLLOWING MANNER:
//  The INNER white quadrant is situated in the first quadrant of the x-y plane
//  with z pointing out of the fiducial. 
//-----------------------------------------------------------------------------

fiducial_detector_t* fiducial_detector_alloc()
{
  fiducial_detector_t* self;

  self = (fiducial_detector_t*) calloc(1,sizeof(fiducial_detector_t));
  assert(self);
   
  return self;
}

void fiducial_detector_free(fiducial_detector_t* self)
{

  if(self)
    free(self);

  return;
}

int world_point_reproject(fiducial_stereo_cam_model_t *camera, double world_point[3], double image_point[2])
{
  fiducial_pose_t transform = fiducial_pose_from_transform(camera->inv_transform);
  fiducial_vec_t w_pt = fiducial_vec_set(world_point[0], world_point[1], world_point[2]);
  fiducial_vec_t new_pt = fiducial_vec_transform(transform, w_pt);

  image_point[0] = camera->focal_length_x * new_pt.x / new_pt.z + camera->image_center_x;
  image_point[1] = camera->focal_length_x * new_pt.y / new_pt.z + camera->image_center_y;
  
  return 0;
}
  
void fiducial_detector_get_params(fiducial_detector_t* self, fiducial_params_t* params)
{
  memcpy(params, &self->params, sizeof(fiducial_params_t));
  return;
}

void fiducial_detector_set_params(fiducial_detector_t* self, const fiducial_params_t* params)
{
  memcpy(&self->params, params, sizeof(fiducial_params_t));
  return;
}

void fiducial_detector_generate_template(fiducial_detector_t* self)
{

  //-----------------------------------------------------------------------------
  //  THE FIDUCIAL TEMPLATE IS ORIENTED IN THE FOLLOWING MANNER:
  //  The INNER white quadrant is situated in the first quadrant of the x-y plane
  //  with z pointing out of the fiducial. 
  //-----------------------------------------------------------------------------
  if(self->template_generated)
    return;

  int i = 0;
  double theta;
  self->num_inner_circle_pts = 0;
  self->num_outer_circle_pts = 0;

  for(i = 0; i < FIDUCIAL_DETECTOR_MAX_INNER_PTS; i++)
  {
    theta = 2*M_PI/(FIDUCIAL_DETECTOR_MAX_INNER_PTS) * (i);
    
    if(theta != 0.0 && theta != M_PI/2.0 && theta != M_PI && theta != 3.0*M_PI/2.0 && theta != 2*M_PI)
    {
      self->inner_circle_pts[self->num_inner_circle_pts].x = FIDUCIAL_DETECTOR_INNER_RADIUS*cos(theta);
      self->inner_circle_pts[self->num_inner_circle_pts].y = FIDUCIAL_DETECTOR_INNER_RADIUS*sin(theta);
      self->inner_circle_pts[self->num_inner_circle_pts].z = 0.0;

      if((theta > 0.0) && (theta < M_PI/2.0))
      {
        self->inner_sign[self->num_inner_circle_pts] = -1;
      }
      else if((theta > M_PI/2.0) && (theta < M_PI))
      {
        self->inner_sign[self->num_inner_circle_pts] = 1;
      }
      else if((theta > M_PI) && (theta < 3.0*M_PI/2.0))
      {
        self->inner_sign[self->num_inner_circle_pts] = -1;
      }
      else if((theta > 3.0*M_PI/2.0) && (theta < 2.0*M_PI))
      {
        self->inner_sign[self->num_inner_circle_pts] = 1;
      }

      self->num_inner_circle_pts =  self->num_inner_circle_pts + 1;
      //fprintf(stderr,"%f %f\n", self->inner_circle_pts[self->num_inner_circle_pts].x, self->inner_circle_pts[self->num_inner_circle_pts].y);
    }
  }

  for(i = 0; i < FIDUCIAL_DETECTOR_MAX_OUTER_PTS/2; i++)
  {
    theta =  M_PI/2.0  + (M_PI/2.0) / (FIDUCIAL_DETECTOR_MAX_OUTER_PTS/2) * i;
     
    if(theta != M_PI/2.0 && theta != M_PI)
    {
      self->outer_circle_pts[self->num_outer_circle_pts].x = FIDUCIAL_DETECTOR_OUTER_RADIUS*cos(theta);
      self->outer_circle_pts[self->num_outer_circle_pts].y = FIDUCIAL_DETECTOR_OUTER_RADIUS*sin(theta);
      self->outer_circle_pts[self->num_outer_circle_pts].z = 0.0;
      self->outer_sign[self->num_outer_circle_pts] = -1;
      self->num_outer_circle_pts = self->num_outer_circle_pts + 1;
      //fprintf(stderr,"%f %f\n", self->outer_circle_pts[self->num_outer_circle_pts].x, self->outer_circle_pts[self->num_outer_circle_pts].y);
    }
  }

  for(i = 0; i < FIDUCIAL_DETECTOR_MAX_OUTER_PTS/2; i++)
  {
    theta = 3.0*M_PI/2.0 + (M_PI/2.0) / (FIDUCIAL_DETECTOR_MAX_OUTER_PTS/2) * i;
     
    if(theta != 3.0*M_PI/2.0 && theta != 2.0*M_PI)
    {
      self->outer_circle_pts[self->num_outer_circle_pts].x = FIDUCIAL_DETECTOR_OUTER_RADIUS*cos(theta);
      self->outer_circle_pts[self->num_outer_circle_pts].y = FIDUCIAL_DETECTOR_OUTER_RADIUS*sin(theta);
      self->outer_circle_pts[self->num_outer_circle_pts].z = 0.0;
      self->outer_sign[self->num_outer_circle_pts] = -1;
      self->num_outer_circle_pts = self->num_outer_circle_pts + 1;
      //fprintf(stderr,"%f %f\n", self->outer_circle_pts[self->num_outer_circle_pts].x, self->outer_circle_pts[self->num_outer_circle_pts].y);
    }
  }

  int t;
  for(t = 0; t < 4; t++)
  {
    theta = M_PI/2.0*t;
    for(i = 0; i < FIDUCIAL_DETECTOR_MAX_EDGE_PTS/4; i++)
    {
      double radius = FIDUCIAL_DETECTOR_OUTER_RADIUS / (FIDUCIAL_DETECTOR_MAX_EDGE_PTS/4) * i;

      if((radius != 0.0) && (radius != FIDUCIAL_DETECTOR_INNER_RADIUS) && (radius != FIDUCIAL_DETECTOR_OUTER_RADIUS))
      {
        self->edge_pts[self->num_edge_pts].x = radius*cos(theta);
        self->edge_pts[self->num_edge_pts].y = radius*sin(theta);

        if((theta == 0.0) || (theta == M_PI))
        {
          if(radius < FIDUCIAL_DETECTOR_INNER_RADIUS)
          {
            self->edge_sign[self->num_edge_pts] = -1;
          }
          else if(radius > FIDUCIAL_DETECTOR_INNER_RADIUS)
          {
            self->edge_sign[self->num_edge_pts] = 1;
          }
        }
        else if((theta == M_PI/2.0) || (theta == 3.0*M_PI/2.0))
        {
          if(radius < FIDUCIAL_DETECTOR_INNER_RADIUS)
          {
            self->edge_sign[self->num_edge_pts] = 1;
          }
          else if(radius > FIDUCIAL_DETECTOR_INNER_RADIUS)
          {
            self->edge_sign[self->num_edge_pts] = -1;
          }
        }

        self->num_edge_pts = self->num_edge_pts + 1;
      }

    }
  }

  self->template_generated = 1;

  return;
}

void fiducial_detector_init(fiducial_detector_t* self)
{
  self->num_inner_circle_pts = 0;
  self->num_outer_circle_pts = 0;
  self->template_generated = 0;
  self->camera_models_set = 0;
  self->fiducial_projected = 0;

  self->initial_fiducial_pose = fiducial_pose_ident();
  self->fiducial_pose = fiducial_pose_ident();

  // set params
  self->params.search_size = FIDUCIAL_DETECTOR_SEARCH_SIZE;
  self->params.min_viewing_angle = 20.0; // degrees
  self->params.dist_thresh = 0.1;
 
  
  // generate template
  fiducial_detector_generate_template(self);

  return;
}

fiducial_detector_error_t fiducial_detector_set_camera_models(fiducial_detector_t* self,
                                                              fiducial_stereo_cam_model_t* rect_model) // must be linearized model
{
  if(self->camera_models_set)
    return FIDUCIAL_DETECTOR_OK;

  memcpy(&self->camera,  rect_model,  sizeof(fiducial_stereo_cam_model_t));
  fiducial_pose_t transform;
  transform = fiducial_pose_from_transform(self->camera.transform);
  transform = fiducial_pose_inv(transform);
  fiducial_pose_to_transform(transform, self->camera.inv_transform);

  self->camera_models_set = 1;
  
  return FIDUCIAL_DETECTOR_OK;   
}


fiducial_detector_error_t fiducial_detector_project_fiducial(fiducial_detector_t* self, fiducial_pose_t fd_pose)
{
 
  
  if(!self->camera_models_set)
  {
    fprintf(stderr,"camera models not set\n");
    return FIDUCIAL_DETECTOR_ERR;
  }

  // reset projection
  self->fiducial_projected = 0;

  // Check if fiducial is visible
  double T[4][4];
  fiducial_pose_to_transform(fd_pose, T);
  fiducial_vec_t camera_to_fiducial = fiducial_vec_unit(fd_pose.pos);
  fiducial_vec_t fiducial_normal = {T[0][2], T[1][2], T[2][2]};

  double angle = 180 * acos( fiducial_vec_dot(camera_to_fiducial,fiducial_normal) / ( fiducial_vec_mag(fiducial_normal) * fiducial_vec_mag(camera_to_fiducial) ) ) / M_PI;

  double min_viewing_angle = self->params.min_viewing_angle;
  if( (angle > (270 + min_viewing_angle) ) || (angle < (90 + min_viewing_angle)) )
  {
    return FIDUCIAL_DETECTOR_ERR;
  }

  int in_bounds;
  int err;
  double col;
  double row;
  int cols, rows;
  double image_point[2];
  double world_point[3];
  fiducial_vec_t world_template_pt;

  //-----------------------------------------------------------------------------
  //  inner circle
  //-----------------------------------------------------------------------------
  // get image dimensions
  cols = self->camera.cols;
  rows = self->camera.rows;


  // Center point
  fiducial_vec_t center_pt = {0,0,0};
  world_template_pt = fiducial_vec_transform(fd_pose, center_pt);
  world_point[0] = world_template_pt.x;
  world_point[1] = world_template_pt.y;
  world_point[2] = world_template_pt.z;

  // reproject the point into the image frame
  err = world_point_reproject(&self->camera, &world_point[0], &image_point[0]);
  col = image_point[0];
  row = image_point[1];

  if( ((int)col < 0) || ((int)col > (cols)) ||
      ((int)row < 0) || ((int)row > (rows)) || (err != 0) )
    in_bounds = false;
  else
    in_bounds = true;

  if(in_bounds)
  {
    self->proj_center_pt.x = (double) col;
    self->proj_center_pt.y = (double) row;
  }

  // Remaining points
  int num_proj_inner_circle_pts = 0;
  int i = 0;

  for(i = 0; i < self->num_inner_circle_pts; i++)
  {
    world_template_pt = fiducial_vec_transform(fd_pose, self->inner_circle_pts[i]);
    world_point[0] = world_template_pt.x;
    world_point[1] = world_template_pt.y;
    world_point[2] = world_template_pt.z;

    err = world_point_reproject(&self->camera, &world_point[0], &image_point[0]);
    col = image_point[0];
    row = image_point[1];

    if( ((int)col < 0) || ((int)col > (cols)) ||
        ((int)row < 0) || ((int)row > (rows)) || (err != 0) )
      in_bounds = false;
    else
      in_bounds = true;

    if(in_bounds)
    {
      self->proj_inner_circle_pts[num_proj_inner_circle_pts].x = (double) col;
      self->proj_inner_circle_pts[num_proj_inner_circle_pts].y = (double) row;
      num_proj_inner_circle_pts = num_proj_inner_circle_pts + 1;
              
    }
  }

  if(num_proj_inner_circle_pts != self->num_inner_circle_pts)
  {
    return FIDUCIAL_DETECTOR_ERR;
  }
  else
  {
    self->num_proj_inner_circle_pts = self->num_inner_circle_pts;
  }


  //-----------------------------------------------------------------------------
  //  outer circle
  //-----------------------------------------------------------------------------
  // get image dimensions
  cols = self->camera.cols;
  rows = self->camera.rows;

  // Remaining points
  int num_proj_outer_circle_pts = 0;

  for(i = 0; i < self->num_outer_circle_pts; i++)
  {
    world_template_pt = fiducial_vec_transform(fd_pose, self->outer_circle_pts[i]);
    world_point[0] = world_template_pt.x;
    world_point[1] = world_template_pt.y;
    world_point[2] = world_template_pt.z;

    err = world_point_reproject(&self->camera, &world_point[0], &image_point[0]);
    col = image_point[0];
    row = image_point[1];

    if( ((int)col < 0) || ((int)col > (cols)) ||
        ((int)row < 0) || ((int)row > (rows)) || (err != 0) )
      in_bounds = false;
    else
      in_bounds = true;

    if(in_bounds)
    {
      self->proj_outer_circle_pts[num_proj_outer_circle_pts].x = (double) col;
      self->proj_outer_circle_pts[num_proj_outer_circle_pts].y = (double) row;
      num_proj_outer_circle_pts = num_proj_outer_circle_pts + 1;
    }
  }

  if(num_proj_outer_circle_pts != self->num_outer_circle_pts)
  {
    return FIDUCIAL_DETECTOR_ERR;
  }
  else
  {
    self->num_proj_outer_circle_pts = self->num_outer_circle_pts;
  }


  //-----------------------------------------------------------------------------
  //  edges
  //-----------------------------------------------------------------------------
  // get image dimensions
  cols = self->camera.cols;
  rows = self->camera.rows;

  // Remaining points
  int num_proj_edge_pts = 0;

  for(i = 0; i < self->num_edge_pts; i++)
  {
    world_template_pt = fiducial_vec_transform(fd_pose, self->edge_pts[i]);
    world_point[0] = world_template_pt.x;
    world_point[1] = world_template_pt.y;
    world_point[2] = world_template_pt.z;

    err = world_point_reproject(&self->camera, &world_point[0], &image_point[0]);
    col = image_point[0];
    row = image_point[1];

    if( ((int)col < 0) || ((int)col > (cols)) ||
        ((int)row < 0) || ((int)row > (rows)) || (err != 0) )
      in_bounds = false;
    else
      in_bounds = true;

    if(in_bounds)
    {
      self->proj_edge_pts[num_proj_edge_pts].x = (double) col;
      self->proj_edge_pts[num_proj_edge_pts].y = (double) row;
      num_proj_edge_pts = num_proj_edge_pts + 1;

    }
  }

  if(num_proj_edge_pts != self->num_edge_pts)
  {
    return FIDUCIAL_DETECTOR_ERR;
  }
  else
  {
    self->num_proj_edge_pts = self->num_edge_pts;
  }


  self->fiducial_projected = 1;

  return FIDUCIAL_DETECTOR_OK;
}

fiducial_detector_error_t fiducial_detector_draw_fiducial(fiducial_detector_t* self,
                                                          uint8_t* image_data,
                                                          int cols, int rows, int channels)
{

  if(channels != 3)
    return FIDUCIAL_DETECTOR_ERR;
  
  uint8_t color[3];
  int m;
  for(m=0; m<2; m++)
  {
    if(m==0)
    {
      // draw the estimated pose first
      fiducial_detector_project_fiducial(self, self->fiducial_pose);
      color[0] = 255;
      color[1] = 0;
      color[2] = 255;
    }
    else
    {      
      // draw the kinematic pose second
      fiducial_detector_project_fiducial(self, self->initial_fiducial_pose);
      color[0] = 0;
      color[1] = 0;
      color[2] = 255;
    }

    int i = 0;
    int n = 0;
    int col, row;
    for(i = 0; i < self->num_proj_inner_circle_pts; i++)
    {
      col = self->proj_inner_circle_pts[i].x;
      row = self->proj_inner_circle_pts[i].y;
      for(n=0;n<1;n++)
      {
        image_data[3*((row+n)*cols + col) + 0] = color[0];
        image_data[3*((row+n)*cols + col) + 1] = color[1];
        image_data[3*((row+n)*cols + col) + 2] = color[2];

        image_data[3*((row)*cols + col+n) + 0] = color[0];
        image_data[3*((row)*cols + col+n) + 1] = color[1];
        image_data[3*((row)*cols + col+n) + 2] = color[2];

      }
    }
    for(i = 0; i < self->num_proj_outer_circle_pts; i++)
    {
      col = self->proj_outer_circle_pts[i].x;
      row = self->proj_outer_circle_pts[i].y;
      for(n=0;n<1;n++)
      {
        image_data[3*((row+n)*cols + col) + 0] = color[0];
        image_data[3*((row+n)*cols + col) + 1] = color[1];
        image_data[3*((row+n)*cols + col) + 2] = color[2];

        image_data[3*((row)*cols + col+n) + 0] = color[0];
        image_data[3*((row)*cols + col+n) + 1] = color[1];
        image_data[3*((row)*cols + col+n) + 2] = color[2];
      }
    }

    for(i = 0; i < self->num_proj_edge_pts; i++)
    {
      col = self->proj_edge_pts[i].x;
      row = self->proj_edge_pts[i].y;
      for(n=0;n<1;n++)
      {
        image_data[3*((row+n)*cols + col) + 0] = color[0];
        image_data[3*((row+n)*cols + col) + 1] = color[1];
        image_data[3*((row+n)*cols + col) + 2] = color[2];

        image_data[3*((row)*cols + col+n) + 0] = color[0];
        image_data[3*((row)*cols + col+n) + 1] = color[1];
        image_data[3*((row)*cols + col+n) + 2] = color[2];
      }
    }
  }    

  return FIDUCIAL_DETECTOR_OK;
}

/// Draw the fiducials in a debug image 
fiducial_detector_error_t fiducial_detector_draw_debug(fiducial_detector_t* self,
                                                       uint8_t* image_data,
                                                       int cols, int rows, int channels,
                                                       fiducial_pose_t fiducial_pose,
                                                       uint8_t color[3])
{
  if(channels != 3)
    return FIDUCIAL_DETECTOR_ERR;
   
  // draw the pose 
  fiducial_detector_project_fiducial(self, fiducial_pose);
  
  int i = 0;
  int n = 0;
  int col, row;
  for(i = 0; i < self->num_proj_inner_circle_pts; i++)
  {
    col = self->proj_inner_circle_pts[i].x;
    row = self->proj_inner_circle_pts[i].y;
    for(n=0;n<1;n++)
    {
      image_data[3*((row+n)*cols + col) + 0] = color[0];
      image_data[3*((row+n)*cols + col) + 1] = color[1];
      image_data[3*((row+n)*cols + col) + 2] = color[2];

      image_data[3*((row)*cols + col+n) + 0] = color[0];
      image_data[3*((row)*cols + col+n) + 1] = color[1];
      image_data[3*((row)*cols + col+n) + 2] = color[2];
    }
  }
  for(i = 0; i < self->num_proj_outer_circle_pts; i++)
  {
    col = self->proj_outer_circle_pts[i].x;
    row = self->proj_outer_circle_pts[i].y;
    for(n=0;n<1;n++)
    {
      image_data[3*((row+n)*cols + col) + 0] = color[0];
      image_data[3*((row+n)*cols + col) + 1] = color[1];
      image_data[3*((row+n)*cols + col) + 2] = color[2];

      image_data[3*((row)*cols + col+n) + 0] = color[0];
      image_data[3*((row)*cols + col+n) + 1] = color[1];
      image_data[3*((row)*cols + col+n) + 2] = color[2];
    }
  }

  for(i = 0; i < self->num_proj_edge_pts; i++)
  {
    col = self->proj_edge_pts[i].x;
    row = self->proj_edge_pts[i].y;
    for(n=0;n<1;n++)
    {
      image_data[3*((row+n)*cols + col) + 0] = color[0];
      image_data[3*((row+n)*cols + col) + 1] = color[1];
      image_data[3*((row+n)*cols + col) + 2] = color[2];

      image_data[3*((row)*cols + col+n) + 0] = color[0];
      image_data[3*((row)*cols + col+n) + 1] = color[1];
      image_data[3*((row)*cols + col+n) + 2] = color[2];
    }
  }

  return FIDUCIAL_DETECTOR_OK;
}


float fiducial_subpix(const uint8_t* image_data, int cols, int rows, int channels, float c, float r)
{
  int c0, r0, c1, r1;
  float c_res, r_res;
  const unsigned char *v00, *v01, *v10, *v11;
  float v0, v1, v;
  int ch;
  
  c0 = floor(c);
  r0 = floor(r);
  c1 = c0 + 1;
  r1 = r0 + 1;
  c_res = c - c0;
  r_res = r - r0;

  if((c1 >= cols) || (r1 >= rows))
    return -1;    

  v00 = &image_data[channels*(r0*cols + c0 )];
  v01 = &image_data[channels*(r1*cols + c0 )];
  v10 = &image_data[channels*(r0*cols + c1 )];
  v11 = &image_data[channels*(r1*cols + c1 )];

  v = 0;
  
  for(ch=0; ch<channels; ch++)
  {
    v0 = (1 - c_res) * v00[ch] + c_res * v10[ch];
    v1 = (1 - c_res) * v01[ch] + c_res * v11[ch];
    
    v += (1 - r_res) * v0 + r_res * v1;
  }
  
  return v / channels;
}

fiducial_detector_error_t fiducial_detector_match(fiducial_detector_t* self,
                                                  uint8_t* image_data,
                                                  int cols, int rows, int channels,
                                                  fiducial_pose_t initial_fiducial_pose,
                                                  float* score)
{  
  if(channels != 3)
    return FIDUCIAL_DETECTOR_ERR;

  // Store starting initial pose
  self->initial_fiducial_pose = initial_fiducial_pose;

  // Project fiducial with initial pose
  if( fiducial_detector_project_fiducial(self, initial_fiducial_pose) != FIDUCIAL_DETECTOR_OK)
  {
    return FIDUCIAL_DETECTOR_ERR;
  }

  int ch;
  int c, r;
  int first_col;
  int first_row;
  int second_col;
  int second_row;
  int i = 0;
  float value_1;
  fiducial_vec2_t radius, first_pt, second_pt;

  float value;
  float max_value;
  

  memset(&self->score, 0, sizeof(float)*2*FIDUCIAL_DETECTOR_SEARCH_SIZE*2*FIDUCIAL_DETECTOR_SEARCH_SIZE);
  //-----------------------------------------------------------------------------
  //  inner circle
  //-----------------------------------------------------------------------------
  for(i = 0; i < self->num_proj_inner_circle_pts; i++)
  {

    radius = fiducial_vec2_sub(self->proj_inner_circle_pts[i], self->proj_center_pt);
    radius = fiducial_vec2_scale(radius, 1.0/fiducial_vec2_mag(radius));
        
    first_pt = fiducial_vec2_add(self->proj_inner_circle_pts[i], radius);
    second_pt = fiducial_vec2_sub(self->proj_inner_circle_pts[i], radius);

    first_col = round(first_pt.x);
    first_row = round(first_pt.y);
    second_col = round(second_pt.x);
    second_row = round(second_pt.y);

    for(r = -self->params.search_size; r < self->params.search_size; r++)
    {
      for(c = -self->params.search_size; c < self->params.search_size; c++)
      {

        uint8_t* image_first_pt = &image_data[channels*((first_row + r)*cols + first_col + c )];
        uint8_t* image_second_pt = &image_data[channels*((second_row + r)*cols + second_col + c )];
        value_1 = 0;
        for(ch = 0; ch < channels; ch++)
        {
          value_1 += image_first_pt[ch] - image_second_pt[ch];
        }
        value = self->inner_sign[i]*(value_1);
        // NOTE: I do not average over the channels for computation speed's sake
        

        self->score[(r+self->params.search_size)*(2*self->params.search_size) + (c+self->params.search_size)] += value;
      }
    }
  }

  //-----------------------------------------------------------------------------
  // edges  
  //-----------------------------------------------------------------------------
  for(i = 0; i < self->num_proj_edge_pts; i++)
  {
    radius = fiducial_vec2_sub(self->proj_edge_pts[i], self->proj_center_pt);
    radius = fiducial_vec2_scale(radius, 1.0/fiducial_vec2_mag(radius) );
    radius = fiducial_vec2_rotate(radius, M_PI/2.0 );

    first_pt = fiducial_vec2_add(self->proj_edge_pts[i], radius);
    second_pt = fiducial_vec2_sub(self->proj_edge_pts[i], radius);

    first_col = round(first_pt.x);
    first_row = round(first_pt.y);
    second_col = round(second_pt.x);
    second_row = round(second_pt.y);

    for(r = -self->params.search_size; r < self->params.search_size; r++)
    {

      for(c = -self->params.search_size; c < self->params.search_size; c++)
      {

        uint8_t* image_first_pt = &image_data[channels*((first_row + r)*cols + first_col + c )];
        uint8_t* image_second_pt = &image_data[channels*((second_row + r)*cols + second_col + c )];
        value_1 = 0;
        for(ch = 0; ch < channels; ch++)
        {
          value_1 += image_first_pt[ch] - image_second_pt[ch];
        }
        value = self->edge_sign[i]*(value_1);
        // NOTE: I do not average over the channels for computation speed's sake

        self->score[(r+self->params.search_size)*(2*self->params.search_size) + (c+self->params.search_size)] += value;
      }
    }
  }


  max_value = FLT_MIN;
  for(r = -self->params.search_size; r < self->params.search_size; r++)
  {
    for(c = -self->params.search_size; c < self->params.search_size; c++)
    {
      value = self->score[(r+self->params.search_size)*(2*self->params.search_size) + (c+self->params.search_size)];

      if(value > max_value)
      {
        max_value = value;
        self->fiducial_location_delta.x = (double) c;
        self->fiducial_location_delta.y = (double) r;
      }
    }
  }

  //fprintf(stderr," best pix r: %f c: %f\n", self->fiducial_location_delta.y, self->fiducial_location_delta.x);
  self->fiducial_location = fiducial_vec2_add(self->proj_center_pt, self->fiducial_location_delta);
  //fprintf(stderr," location before subpix r: %f c: %f\n", self->fiducial_location.y, self->fiducial_location.x);
  *score = max_value;

  return FIDUCIAL_DETECTOR_OK;
}

fiducial_detector_error_t fiducial_detector_match_subpixel(fiducial_detector_t* self,
                                                            uint8_t* image_data,
                                                            int cols, int rows, int channels,
                                                            fiducial_pose_t initial_fiducial_pose,
                                                            float* score)
{

  if(channels != 3)
    return FIDUCIAL_DETECTOR_ERR;

  // store initial fiducial pose
  self->initial_fiducial_pose = initial_fiducial_pose;

  // Match at the pixel level first
  if( fiducial_detector_match(self, image_data, cols, rows, channels, initial_fiducial_pose, score) != FIDUCIAL_DETECTOR_OK)
    return FIDUCIAL_DETECTOR_ERR;

  int c, r;
  float first_col;
  float first_row;
  float second_col;
  float second_row;
  int i = 0;
  fiducial_vec2_t radius, first_pt, second_pt;

  float value;
  float max_value;
 
  memset(&self->subpix_score, 0, sizeof(float)*FIDUCIAL_DETECTOR_MAX_SUBPIXELS*FIDUCIAL_DETECTOR_MAX_SUBPIXELS);
  //-----------------------------------------------------------------------------
  //  inner circle
  //-----------------------------------------------------------------------------
  for(i = 0; i < self->num_proj_inner_circle_pts; i++)
  {

    radius = fiducial_vec2_sub(self->proj_inner_circle_pts[i], self->proj_center_pt);
    radius = fiducial_vec2_scale(radius, 1.0/fiducial_vec2_mag(radius));

    first_pt = fiducial_vec2_add(self->proj_inner_circle_pts[i], radius);
    second_pt = fiducial_vec2_sub(self->proj_inner_circle_pts[i], radius);

    first_col = (first_pt.x);
    first_row = (first_pt.y);
    second_col = (second_pt.x);
    second_row = (second_pt.y);

    for(r = 0; r < FIDUCIAL_DETECTOR_MAX_SUBPIXELS; r++)
    {
      for(c = 0; c < FIDUCIAL_DETECTOR_MAX_SUBPIXELS; c++)
      {
        float rf = r/((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS) - 0.5 + self->fiducial_location_delta.y;
        float cf = c/((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS) - 0.5 + self->fiducial_location_delta.x;

        float subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col + cf, first_row + rf);
        float subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col + cf, second_row + rf);
        value = self->inner_sign[i]*(subpix_image_first_pt - subpix_image_second_pt);

        self->subpix_score[r*FIDUCIAL_DETECTOR_MAX_SUBPIXELS + c] += value;
      }
    }
  }
  //-----------------------------------------------------------------------------
  // edges  
  //-----------------------------------------------------------------------------
  for(i = 0; i < self->num_proj_edge_pts; i++)
  {
    radius = fiducial_vec2_sub(self->proj_edge_pts[i], self->proj_center_pt);
    radius = fiducial_vec2_scale(radius, 1.0/fiducial_vec2_mag(radius) );
    radius = fiducial_vec2_rotate(radius, M_PI/2.0 );

    first_pt = fiducial_vec2_add(self->proj_edge_pts[i], radius);
    second_pt = fiducial_vec2_sub(self->proj_edge_pts[i], radius);

    first_col = (first_pt.x);
    first_row = (first_pt.y);
    second_col = (second_pt.x);
    second_row = (second_pt.y);

    for(r = 0; r < FIDUCIAL_DETECTOR_MAX_SUBPIXELS; r++)
    {
      for(c = 0; c < FIDUCIAL_DETECTOR_MAX_SUBPIXELS; c++)
      {
        float rf = r/((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS) - 0.5 + self->fiducial_location_delta.y;
        float cf = c/((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS) - 0.5 + self->fiducial_location_delta.x;

        float subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col + cf, first_row + rf);
        float subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col + cf, second_row + rf);
        value = self->edge_sign[i]*(subpix_image_first_pt - subpix_image_second_pt);

        self->subpix_score[r*FIDUCIAL_DETECTOR_MAX_SUBPIXELS + c] += value;
      }
    }

  }

  // Get best row and col
  max_value = FLT_MIN;
  float delta_c = 0;
  float delta_r = 0;
  for(r = 0; r < FIDUCIAL_DETECTOR_MAX_SUBPIXELS; r++)
  {
    for(c = 0; c < FIDUCIAL_DETECTOR_MAX_SUBPIXELS; c++)
    {

      value = self->subpix_score[r*FIDUCIAL_DETECTOR_MAX_SUBPIXELS+c];

      if(value > max_value)
      {
        max_value = value;
        delta_r = (float) (r/((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS) - 0.5 + self->fiducial_location_delta.y);
        delta_c = (float) (c/((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS) - 0.5 + self->fiducial_location_delta.x);
      }
    }
  }

//    fprintf(stderr," best subpix r: %f c: %f\n", delta_r, delta_c);
    self->fiducial_location_delta.y = delta_r;
    self->fiducial_location_delta.x = delta_c;
    self->fiducial_location = fiducial_vec2_add(self->proj_center_pt, self->fiducial_location_delta);
//    fprintf(stderr," location after subpix r: %f c: %f\n", self->fiducial_location.y, self->fiducial_location.x);

  *score = max_value;

  return FIDUCIAL_DETECTOR_OK;
}

fiducial_detector_error_t fiducial_detector_gradient_descent(  fiducial_detector_t* self, 
                                                               uint8_t* image_data,
                                                               int cols, int rows, int channels,
                                                               int max_iterations,
                                                               float translation_tolerance,
                                                               float angular_tolerance,
                                                               fiducial_pose_t *fd_pose,
                                                               float* score)
{
  if(channels != 3)
    return FIDUCIAL_DETECTOR_ERR;

  if(!self->fiducial_projected)
    return FIDUCIAL_DETECTOR_ERR;

  int debug = 0;

  // Main Equation
  // Maximizing S = \sum_x( del I(x) ) \approx \sum_x( I_{outer}(x) - I_{inner}(x) )
  // where x are the templates points in image coordinates. 
  // I_{outer} and I_{inner} are the image pixels
  // on outside and inside of the template to approximate the gradient. 
  // The gradient descent equation used is:
  // X_new = X_old - gamma*delS/dX
  // where delS/dX = \sum_x delI*dx/dX
  // where X is the pose (6DOF) 

  // Store starting pose
  fiducial_pose_t current_pose = *fd_pose;

  int i;
  int iter;
  int cont_iter = 0;
  int col;
  int row;
  int inner;
  float first_col;
  float first_row;
  float second_col;
  float second_row;
  float subpix_image_first_pt;
  float subpix_image_second_pt;

  fiducial_vec2_t radius;
  fiducial_vec2_t outer_pt;
  fiducial_vec2_t inner_pt;

  float Iouter[1][2] = {{0.0}};
  float Iinner[1][2] = {{0.0}};

  float D[2][6] = {{0.0}};
  float X[1][6] = {{0.0}}; 
  
  double r,p,y;
  X[0][0] = current_pose.pos.x;
  X[0][1] = current_pose.pos.y;
  X[0][2] = current_pose.pos.z;
  fiducial_rot_to_rpy(current_pose.rot, &r, &p, &y); 
  X[0][3] = (float) r; 
  X[0][4] = (float) p;
  X[0][5] = (float) y;
  
  if(debug)
  {
    fprintf(stderr,"x before: %f\n", current_pose.pos.x);
    fprintf(stderr,"y before: %f\n", current_pose.pos.y);
    fprintf(stderr,"z before: %f\n", current_pose.pos.z);
    fprintf(stderr,"r before: %lf\n", 180.0/M_PI*r);
    fprintf(stderr,"p before: %lf\n", 180.0/M_PI*p);
    fprintf(stderr,"y before: %lf\n", 180.0/M_PI*y);
  }

  fiducial_vec_t vec_temp;
  fiducial_vec_t i_vec = fiducial_vec_set(1.0,0.0,0.0);
  fiducial_vec_t j_vec = fiducial_vec_set(0.0,1.0,0.0);
  fiducial_vec_t k_vec = fiducial_vec_set(0.0,0.0,1.0);
  fiducial_vec_t r_deriv;
  fiducial_vec_t p_deriv;
  fiducial_vec_t y_deriv;

  for(iter = 0; iter < max_iterations; iter++)
  {
    cont_iter = 0;
    // Project fiducial at new pose
    fiducial_detector_project_fiducial(self, current_pose);

    double value = 0;
    double delta;
    double dS_dX[1][6] = {{0.0}};
    // Proportional factor to gradient
    // Change this to alter speed of convergence
    double gamma[6]  = {-5E-10};
    gamma[3] = -5E-7;
    gamma[4] = -5E-7;
    gamma[5] = -5E-7;

    // dx
    D[0][0] = self->camera.focal_length_x / current_pose.pos.z;
    D[1][0] = 0.0;

    // dy
    D[0][1] = 0.0;
    D[1][1] = self->camera.focal_length_y / current_pose.pos.z;

    // dz 
    D[0][2] = - self->camera.focal_length_x * current_pose.pos.x / (current_pose.pos.z*current_pose.pos.z);
    D[1][2] = - self->camera.focal_length_y * current_pose.pos.y / (current_pose.pos.z*current_pose.pos.z);

    // For all the template points lets calculate
    // INNER
    for( i = 0; i < self->num_inner_circle_pts; i++)
    {
      vec_temp = fiducial_vec_rotate(current_pose.rot, self->inner_circle_pts[i]);

      r_deriv = fiducial_vec_cross(i_vec, vec_temp);
      p_deriv = fiducial_vec_cross(j_vec, vec_temp);
      y_deriv = fiducial_vec_cross(k_vec, vec_temp);


      D[0][3] = self->camera.focal_length_x / (current_pose.pos.z*current_pose.pos.z) * (r_deriv.x* current_pose.pos.z - current_pose.pos.x * r_deriv.z);
      D[1][3] = self->camera.focal_length_y / (current_pose.pos.z*current_pose.pos.z) * (r_deriv.y* current_pose.pos.z - current_pose.pos.y * r_deriv.z);

      D[0][4] = self->camera.focal_length_x / (current_pose.pos.z*current_pose.pos.z) * (p_deriv.x* current_pose.pos.z - current_pose.pos.x * p_deriv.z);
      D[1][4] = self->camera.focal_length_y / (current_pose.pos.z*current_pose.pos.z) * (p_deriv.y* current_pose.pos.z - current_pose.pos.y * p_deriv.z);

      D[0][5] = self->camera.focal_length_x / (current_pose.pos.z*current_pose.pos.z) * (y_deriv.x* current_pose.pos.z - current_pose.pos.x * y_deriv.z);
      D[1][5] = self->camera.focal_length_y / (current_pose.pos.z*current_pose.pos.z) * (y_deriv.y* current_pose.pos.z - current_pose.pos.y * y_deriv.z);


      radius = fiducial_vec2_sub(self->proj_inner_circle_pts[i], self->proj_center_pt);
      radius = fiducial_vec2_scale(radius, 0.5/fiducial_vec2_mag(radius));

      outer_pt = fiducial_vec2_add(self->proj_inner_circle_pts[i], radius);
      inner_pt = fiducial_vec2_sub(self->proj_inner_circle_pts[i], radius);

      // 
      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, outer_pt.x, outer_pt.y);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, inner_pt.x, inner_pt.y);
      value = value + (double) self->inner_sign[i]*(subpix_image_first_pt - subpix_image_second_pt);


      // First calculate image gradient!
      // Outer
      first_col = outer_pt.x - 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      first_row = outer_pt.y; 
      second_col = outer_pt.x + 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      second_row = outer_pt.y;

      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col, first_row);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col, second_row);

      Iouter[0][0] = (subpix_image_second_pt - subpix_image_first_pt) / (2.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS));

      first_col = outer_pt.x;
      first_row = outer_pt.y - 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      second_col = outer_pt.x;
      second_row = outer_pt.y + 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  

      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col, first_row);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col, second_row);

      Iouter[0][1] = (subpix_image_second_pt - subpix_image_first_pt) / (2.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS));

      // inner
      first_col = inner_pt.x - 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      first_row = inner_pt.y;
      second_col = inner_pt.x + 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      second_row = inner_pt.y;

      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col, first_row);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col, second_row);

      Iinner[0][0] = (subpix_image_second_pt - subpix_image_first_pt) / (2.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS));

      first_col = inner_pt.x;
      first_row = inner_pt.y - 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      second_col = inner_pt.x;
      second_row = inner_pt.y + 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  

      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col, first_row);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col, second_row);

      Iinner[0][1] = (subpix_image_second_pt - subpix_image_first_pt) / (2.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS));
      /*
      for(row = 0; row < 1; row++)
      {
        for(col = 0; col < 6; col++)
        {
          del_dS_dX[row][col] = 0;
        }
      }
      */

      for(row = 0; row < 1; row++)
      {
        for(col = 0; col < 6; col++)
        {
          for(inner = 0; inner < 2; inner++)
          {
            dS_dX[row][col] = dS_dX[row][col] + self->inner_sign[i]*(Iouter[row][inner]*D[inner][col] - Iinner[row][inner]*D[inner][col]);
          }
        }  
      }
      /* 
      fprintf(stderr,"sign: %d\n",self->inner_sign[i]);
      fprintf(stderr,"I's\n");
      fprintf(stderr,"%f %f     %f %f\n", Iouter[0][0], Iouter[0][1], Iinner[0][0], Iinner[0][1]);
      fprintf(stderr,"D\n");
      for(row = 0; row < 2; row++)
      {
        for(col = 0; col < 6; col++)
        {
          fprintf(stderr,"%f ",D[row][col]);
        }
        fprintf(stderr,"\n");
      }

      fprintf(stderr,"del_S_dX\n");
      for(row = 0; row < 1; row++)
      {
        for(col = 0; col < 6; col++)
        {
          //dS_dX[row][col] += del_dS_dX[row][col];
          fprintf(stderr,"%f ",dS_dX[row][col]);
        }
        fprintf(stderr,"\n");
      }
      */      

    }



    
    // EDGE points
    for(i = 0; i < self->num_proj_edge_pts; i++)
    {
      vec_temp = fiducial_vec_rotate(current_pose.rot, self->edge_pts[i]);

      r_deriv = fiducial_vec_cross(i_vec, vec_temp);
      p_deriv = fiducial_vec_cross(j_vec, vec_temp);
      y_deriv = fiducial_vec_cross(k_vec, vec_temp);


      D[0][3] = self->camera.focal_length_x / (current_pose.pos.z*current_pose.pos.z) * (r_deriv.x* current_pose.pos.z - current_pose.pos.x * r_deriv.z);
      D[1][3] = self->camera.focal_length_y / (current_pose.pos.z*current_pose.pos.z) * (r_deriv.y* current_pose.pos.z - current_pose.pos.y * r_deriv.z);

      D[0][4] = self->camera.focal_length_x / (current_pose.pos.z*current_pose.pos.z) * (p_deriv.x* current_pose.pos.z - current_pose.pos.x * p_deriv.z);
      D[1][4] = self->camera.focal_length_y / (current_pose.pos.z*current_pose.pos.z) * (p_deriv.y* current_pose.pos.z - current_pose.pos.y * p_deriv.z);

      D[0][5] = self->camera.focal_length_x / (current_pose.pos.z*current_pose.pos.z) * (y_deriv.x* current_pose.pos.z - current_pose.pos.x * y_deriv.z);
      D[1][5] = self->camera.focal_length_y / (current_pose.pos.z*current_pose.pos.z) * (y_deriv.y* current_pose.pos.z - current_pose.pos.y * y_deriv.z);


      radius = fiducial_vec2_sub(self->proj_edge_pts[i], self->proj_center_pt);
      radius = fiducial_vec2_scale(radius, 0.5/fiducial_vec2_mag(radius) );
      radius = fiducial_vec2_rotate(radius, M_PI/2.0 );

      outer_pt = fiducial_vec2_add(self->proj_edge_pts[i], radius);
      inner_pt = fiducial_vec2_sub(self->proj_edge_pts[i], radius);
      // 
      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, outer_pt.x, outer_pt.y);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, inner_pt.x, inner_pt.y);
      value = value + (double) self->edge_sign[i]*(subpix_image_first_pt - subpix_image_second_pt);


      // First calculate image gradient!
      // Outer
      first_col = outer_pt.x - 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      first_row = outer_pt.y;
      second_col = outer_pt.x + 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      second_row = outer_pt.y;

      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col, first_row);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col, second_row);

      Iouter[0][0] = (subpix_image_second_pt - subpix_image_first_pt) / (2.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS));

      first_col = outer_pt.x;
      first_row = outer_pt.y - 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      second_col = outer_pt.x;
      second_row = outer_pt.y + 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  

      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col, first_row);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col, second_row);

      Iouter[0][1] = (subpix_image_second_pt - subpix_image_first_pt) / (2.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS));

      // inner
      first_col = inner_pt.x - 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      first_row = inner_pt.y;
      second_col = inner_pt.x + 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      second_row = inner_pt.y;

      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col, first_row);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col, second_row);

      Iinner[0][0] = (subpix_image_second_pt - subpix_image_first_pt) / (2.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS));

      first_col = inner_pt.x;
      first_row = inner_pt.y - 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  
      second_col = inner_pt.x;
      second_row = inner_pt.y + 1.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS);  

      subpix_image_first_pt = fiducial_subpix(image_data, cols, rows, channels, first_col, first_row);
      subpix_image_second_pt = fiducial_subpix(image_data, cols, rows, channels, second_col, second_row);

      Iinner[0][1] = (subpix_image_second_pt - subpix_image_first_pt) / (2.0 / ((float) FIDUCIAL_DETECTOR_MAX_SUBPIXELS));

      for(row = 0; row < 1; row++)
      {
        for(col = 0; col < 6; col++)
        {
          for(inner = 0; inner < 2; inner++)
          {
            dS_dX[row][col] = dS_dX[row][col] + self->edge_sign[i]*(Iouter[row][inner]*D[inner][col] - Iinner[row][inner]*D[inner][col]);
          }
        }  
      }



    }
/*
        fprintf(stderr,"dS_dX\n");
      for(row = 0; row < 1; row++)
      {
        for(col = 0; col < 6; col++)
        {
          fprintf(stderr,"%f ",dS_dX[row][col]);
        }
        fprintf(stderr,"\n");
      }
        fprintf(stderr,"D\n");
      for(row = 0; row < 2; row++)
      {
        for(col = 0; col < 6; col++)
        {
          fprintf(stderr,"%f ",D[row][col]);
        }
        fprintf(stderr,"\n");
      }
*/

    // Calculate the delta using the gradient above dS_dX
    for(row = 0; row < 1; row++)
    {
      for(col = 0; col < 6; col++)
      {
        delta = gamma[col]*dS_dX[row][col];

        // Update pose with delta
        X[row][col] = X[row][col] - delta;

        // Check stopping criterion
        if(col < 3) // Translation
        {
          if(fabs(delta) > translation_tolerance)
          {
            cont_iter = 1;
          } 
        }
        else // Angular
        {
          if(fabs(delta) > angular_tolerance)
          {
            cont_iter = 1;
          } 
        }
      }
    }
    
    if(debug)
    {
      fprintf(stderr,"iter: %d\n", iter);
      fprintf(stderr,"delta: ");
      for(row = 0; row < 1; row++)
      {
        for(col = 0; col < 6; col++)
        {
          fprintf(stderr,"%f ", gamma[col]*dS_dX[row][col]);
        }
      }
      fprintf(stderr,"\n");
      fprintf(stderr,"X: ");
      for(row = 0; row < 1; row++)
      {
        for(col = 0; col < 6; col++)
        {
          fprintf(stderr,"%f ", X[row][col]);
        }
      }
      fprintf(stderr,"\n");
      fprintf(stderr,"value: %f\n", value);
    }

    // Store current value into a pose 
    current_pose.pos.x = X[0][0];
    current_pose.pos.y = X[0][1];
    current_pose.pos.z = X[0][2];
    r = (double) X[0][3];
    p = (double) X[0][4];
    y = (double) X[0][5];
    current_pose.rot = fiducial_rot_from_rpy(r,p,y);


    if(!cont_iter)
      break;
  }

  if(debug)
  {
    fprintf(stderr,"x after: %f\n", current_pose.pos.x);
    fprintf(stderr,"y after: %f\n", current_pose.pos.y);
    fprintf(stderr,"z after: %f\n", current_pose.pos.z);
    fprintf(stderr,"r after: %lf\n", 180.0/M_PI*r);
    fprintf(stderr,"p after: %lf\n", 180.0/M_PI*p);
    fprintf(stderr,"y after: %lf\n", 180.0/M_PI*y);
  }

  // if exceeds iterations
  if(cont_iter)
  {
    self->fiducial_pose = *fd_pose;
    return FIDUCIAL_DETECTOR_ERR;
  }
  // if greater than max allowable position correction
  if(fiducial_vec_mag(fiducial_vec_sub(fd_pose->pos, current_pose.pos)) > self->params.dist_thresh)
  {
    self->fiducial_pose = *fd_pose;
    return FIDUCIAL_DETECTOR_ERR;
  }

  // All ok, return found pose 
  *fd_pose = current_pose;
  self->fiducial_pose = current_pose;

  fiducial_detector_project_fiducial(self, current_pose);
  
  return FIDUCIAL_DETECTOR_OK;
}
