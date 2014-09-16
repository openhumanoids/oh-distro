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
@file  fiducial_stereo.h
@brief header and macros for stereo msl fiducial detectors

@date  11/27/2012

@author Paul Hebert (paul.hebert@jpl.nasa.gov)
Mobility and Manipulation Group (3475), JPL
*/


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

#include "fiducial_detector.h"

#if !defined(__FIDUCIAL_STEREO_H) 
#define __FIDUCIAL_STEREO_H

#if defined __cplusplus
extern "C"
{
#endif

typedef enum
{
  FIDUCIAL_STEREO_LEFT = 0,
  FIDUCIAL_STEREO_RIGHT,
  FIDUCIAL_STEREO_NUM_CAMERAS
} fiducial_stereo_camera_t;

typedef struct
{
  /// fiducial detector for each camera
  fiducial_detector_t* fiducial_detector[FIDUCIAL_STEREO_NUM_CAMERAS];
  
  /// Initial fiducial pose
  fiducial_pose_t initial_fiducial_pose;

  /// Found fiducial pose
  fiducial_pose_t fiducial_pose;
  
} fiducial_stereo_t;

/// Allocate a fiducial stereo object
fiducial_stereo_t* fiducial_stereo_alloc();

/// Free a fiducial stereo object
void fiducial_stereo_free(fiducial_stereo_t* self);

/// Initialize
void fiducial_stereo_init(fiducial_stereo_t* self);

/// Set the camera models for each detector
fiducial_detector_error_t fiducial_stereo_set_camera_models(fiducial_stereo_t* self, fiducial_stereo_cam_model_t* left_cam, fiducial_stereo_cam_model_t* right_cam );

/// To draw the fiducials in the left and right image
fiducial_detector_error_t fiducial_stereo_draw_fiducials(fiducial_stereo_t* self,
                                                         uint8_t* left_image_data, 
                                                         uint8_t* right_image_data, 
                                                         int image_cols, int image_rows, int image_channels);

/// Process function to detect, match and refine pose of fiducial
fiducial_detector_error_t fiducial_stereo_process(fiducial_stereo_t* self, 
                                                  uint8_t* left_image_data, 
                                                  uint8_t* right_image_data, 
                                                  int image_cols, int image_rows, int image_channels,
                                                  fiducial_pose_t initial_pose,
                                                  fiducial_pose_t *found_pose,
                                                  float *left_score,
                                                  float *right_score,
                                                  bool full_pose); // Want full pose using gradient descent?

#if defined __cplusplus
}
#endif

#endif // __FIDUCIAL_STEREO_H
