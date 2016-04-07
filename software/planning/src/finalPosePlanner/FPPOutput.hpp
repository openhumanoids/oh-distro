/*
 * FPPOutput.hpp
 *
 *  Created on: 23 Mar 2016
 *      Author: marco
 */

#ifndef FPPOUTPUT_HPP_
#define FPPOUTPUT_HPP_

struct FPPOutput
{
  int final_orient;
  int n_valid_samples;
  int n_valid_samples_used;
  double cost;
  int info;
  double computation_time;
  double IK_time;
  double capability_map_time;
  double collision_time;
  double constraints_time;
  double kinematics_time;
  double sampling_time;
  double cm_angle_time;
  double cm_base_hight_time;
  double cm_direction_time;
  double cm_collision_time;
};

#endif /* FPPOUTPUT_HPP_ */
