/*
 * data_fusion_parameters.hpp
 *
 *  Created on: Jun 8, 2013
 *      Author: drc
 */

#ifndef DATA_FUSION_PARAMETERS_HPP_
#define DATA_FUSION_PARAMETERS_HPP_

//0.5, 0.1
#define INS_POS_FEEDBACK_GAIN     0.02

#define INS_POS_WINDUP_BALANCE    0.5

//0.3, 0.9
#define INS_VEL_FEEDBACK_GAIN     0.18

#define INS_VEL_WINDUP_BALANCE    0.9

#define DATA_FUSION_PERIOD      40000
#define ZU_SPEED_THRES            0.01


#endif /* DATA_FUSION_PARAMETERS_HPP_ */
