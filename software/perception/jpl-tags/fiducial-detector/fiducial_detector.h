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
@file  fiducial_detector.h
@brief header and macros for msl fiducial detector

@date  08/29/2012

@author Paul Hebert (paul.hebert@jpl.nasa.gov)
Mobility and Manipulation Group (3475), JPL
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <float.h>


#if !defined(__FIDUCIAL_DETECTOR_H) 
#define __FIDUCIAL_DETECTOR_H

#include "fiducial_pose.h"

//-----------------------------------------------------------------------------
//  THE FIDUCIAL TEMPLATE IS ORIENTED IN THE FOLLOWING MANNER:
//  The INNER white quadrant is situated in the first quadrant of the x-y plane
//  with z pointing out of the fiducial. 
//-----------------------------------------------------------------------------

#define FIDUCIAL_DETECTOR_INNER_RADIUS            0.00635 // METERS
#define FIDUCIAL_DETECTOR_OUTER_RADIUS            0.0127 // METERS
#define FIDUCIAL_DETECTOR_MAX_INNER_PTS           40  // 
#define FIDUCIAL_DETECTOR_MAX_OUTER_PTS           40  // 
#define FIDUCIAL_DETECTOR_MAX_EDGE_PTS            80  // 
#define FIDUCIAL_DETECTOR_MAX_PROJ_PTS            160 // 
#define FIDUCIAL_DETECTOR_SEARCH_SIZE             40  // pixels
#define FIDUCIAL_DETECTOR_MAX_SUBPIXELS           10 


#if defined __cplusplus
extern "C"
{
#endif

typedef enum
{
  FIDUCIAL_DETECTOR_ERR = -1,
  FIDUCIAL_DETECTOR_OK
} fiducial_detector_error_t;

typedef enum
{
  FIDUCIAL_DETECTOR_INNER_CIRCLE = 0,
  FIDUCIAL_DETECTOR_OUTER_CIRCLE,
  FIDUCIAL_DETECTOR_NUM_EDGES
} fiducial_detector_edge_t;

typedef struct
{
  int search_size;          // template search size in pixels
  double min_viewing_angle; // ignore markers that are pitched below this angle of incidence (in degrees)
  double dist_thresh;       // magnitude of the distance between initial to the estimated fiducial location must be smaller than this to be considered valid
} fiducial_params_t;

typedef struct
{
  // num cols of camera
  int cols;

  // num rows of camera
  int rows;

  // focal length in x of the linearized camera model
  double focal_length_x;

  // focal length in y of the linearized camera model
  double focal_length_y;

  // image center in x of the linearized camera model
  double image_center_x;

  // image center in y of the linearized camera model
  double image_center_y;

  // where the camera is relative to the reference camera frame
  // (i.e. LEFT = eye(4), RIGHT = "close to [eye(3) T; zeros(1,3) 1]" where T = [0.12, 0, 0])
  double transform[4][4];

  // Inverse transform of the above (useful for projecting)
  double inv_transform[4][4];

} fiducial_stereo_cam_model_t; // linear (pin-hole)

typedef struct 
{

  /// Template/fiducial points
  fiducial_vec_t inner_circle_pts[FIDUCIAL_DETECTOR_MAX_INNER_PTS];
  int    inner_sign[FIDUCIAL_DETECTOR_MAX_INNER_PTS]; // used for differencing gradient 
  int    num_inner_circle_pts;
  fiducial_vec_t outer_circle_pts[FIDUCIAL_DETECTOR_MAX_OUTER_PTS];
  int    outer_sign[FIDUCIAL_DETECTOR_MAX_OUTER_PTS]; // used for differencing gradient 
  int    num_outer_circle_pts;
  fiducial_vec_t edge_pts[FIDUCIAL_DETECTOR_MAX_EDGE_PTS];
  int    edge_sign[FIDUCIAL_DETECTOR_MAX_EDGE_PTS];
  int    num_edge_pts;
  int template_generated;

  /// Initial Fiducial pose
  fiducial_pose_t initial_fiducial_pose;

  /// Projected template points
  fiducial_vec2_t proj_inner_circle_pts[FIDUCIAL_DETECTOR_MAX_INNER_PTS];
  int    num_proj_inner_circle_pts;
  fiducial_vec2_t proj_outer_circle_pts[FIDUCIAL_DETECTOR_MAX_OUTER_PTS];
  int    num_proj_outer_circle_pts;
  fiducial_vec2_t proj_edge_pts[FIDUCIAL_DETECTOR_MAX_EDGE_PTS];
  int    num_proj_edge_pts;

  fiducial_vec2_t proj_center_pt;

  float score[2*FIDUCIAL_DETECTOR_SEARCH_SIZE*2*FIDUCIAL_DETECTOR_SEARCH_SIZE];
  float subpix_score[10*10];


  int    fiducial_projected;

  
  /// Camera Models
  fiducial_stereo_cam_model_t camera;
  int camera_models_set;
  
  /// Best Location of Fiducial
  fiducial_vec2_t fiducial_location;
  /// Best delta for fiducial
  fiducial_vec2_t fiducial_location_delta;
  /// Fiducial Pose 
  fiducial_pose_t fiducial_pose;

  
  // parameters
  fiducial_params_t params;
} fiducial_detector_t;


  /// Allocates the ::fiducial_detector_t object
  fiducial_detector_t* fiducial_detector_alloc();

  /// Initializes the ::fiducial_detector_t object
  void fiducial_detector_init(fiducial_detector_t* self);

  /// Get the params
  void fiducial_detector_get_params(fiducial_detector_t* self, fiducial_params_t* params);
  
  /// Set the params
  void fiducial_detector_set_params(fiducial_detector_t* self, const fiducial_params_t* params);
  
  /// Frees the ::fiducial_detector_t object
  void fiducial_detector_free(fiducial_detector_t* self);
  
  /// Set the camera model for projection
  fiducial_detector_error_t fiducial_detector_set_camera_models(fiducial_detector_t* self,
                                                                fiducial_stereo_cam_model_t* rect_model); // must be linearized model
  
  /// Project the fiducial into the camera frame
  fiducial_detector_error_t fiducial_detector_project_fiducial(fiducial_detector_t* self, fiducial_pose_t fd_pose);

  /// Draw the fiducials in the image
  fiducial_detector_error_t fiducial_detector_draw_fiducial(fiducial_detector_t* self,
                                                            uint8_t* image_data,
                                                            int cols, int rows, int channels);

  /// Draw the fiducials in a debug image 
  fiducial_detector_error_t fiducial_detector_draw_debug(fiducial_detector_t* self,
                                                         uint8_t* image_data,
                                                         int cols, int rows, int channels,
                                                         fiducial_pose_t fiducial_pose,
                                                         uint8_t color[3]);


  /// Match the fiducial in the left and right image
  fiducial_detector_error_t fiducial_detector_match(fiducial_detector_t* self,
                                                    uint8_t* image_data,
                                                    int cols, int rows, int channels,
                                                    fiducial_pose_t initial_fd_pose,
                                                    float* score);
  /// Match the fiducial in the left and right image at the subpixel level
  fiducial_detector_error_t fiducial_detector_match_subpixel(fiducial_detector_t* self,
                                                             uint8_t* image_data,
                                                             int cols, int rows, int channels,
                                                             fiducial_pose_t initial_fd_pose,
                                                             float* score);

  /// Refine the pose of the fiducial using gradient descent (similar to lucas kanade)
  fiducial_detector_error_t fiducial_detector_gradient_descent(  fiducial_detector_t* self, 
                                                                 uint8_t* image_data,
                                                                 int cols, int rows, int channels,
                                                                 int max_iterations, // max number of iteration for gradient descent
                                                                 float translation_tolerance, // tolerance for x,y,z
                                                                 float angular_tolerance, // tolerance in roll, pitch, yaw
                                                                 fiducial_pose_t *fd_pose, // 
                                                                 float* score);


#if defined __cplusplus
}
#endif


#endif // __FIDUCIAL_DETECTOR_H
