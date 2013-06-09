/*
 * BotTransforms.cpp
 *
 *  Created on: Jun 8, 2013
 *      Author: drc
 */

#include "BotTransforms.hpp"
#include "QuaternionLib.h"
#include <iostream>

BotTransforms::BotTransforms() {
	lc2p.setIdentity();

	std::cout << "BotTransforms object has been created\n";
}

Eigen::Vector3d BotTransforms::lcam2pelvis(const Eigen::Vector3d &vec_cam) {

	Eigen::Isometry3d transform_,ret;

	transform_.setIdentity();
	ret.setIdentity();
	transform_.translation() = vec_cam;

	ret = lc2p * transform_;

	Eigen::Vector3d retvec;

	retvec << transform_.translation().x(),transform_.translation().y(),transform_.translation().z();

	return retvec;
}

void BotTransforms::setLCam2Pelvis(const Eigen::Quaterniond &q,const Eigen::Vector3d &trans) {
	// here we set the transform from left camera frame to the pelvis position.
	// this tranformation is to be done with Eigen::Isometry3d

	lc2p.setIdentity();

	lc2p.translation().x() = trans(0);
	lc2p.translation().y() = trans(1);
	lc2p.translation().z() = trans(2);

	lc2p.linear() = q2C(q);
}

void BotTransforms::setLCam2Pelvis(const Eigen::Isometry3d &c2b) {
	lc2p = c2b;
}


