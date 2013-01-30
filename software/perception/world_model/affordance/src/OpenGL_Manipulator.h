/*
 * OpenGLManipulator.h
 *
 *  Created on: Jan 23, 2013
 *      Author: mfleder
 */

#ifndef OPENGL_MANIPULATOR_H
#define OPENGL_MANIPULATOR_H

//#include "ModelState.h"
#include "ManipulatorState.h"
#include "opengl/opengl_object.h"
#include "opengl/opengl_object_cylinder.h"
#include "opengl/opengl_object_sphere.h"
#include "opengl/opengl_object_box.h"
#include "boost/shared_ptr.hpp"

namespace affordance
{

class OpenGL_Manipulator: public opengl::OpenGL_Object
{
	//------------------fields
private:
	/**underlying manipulator state*/
	const ManipulatorStateConstPtr _manipulator; 

	//-----------------constructors
public:
	OpenGL_Manipulator(ManipulatorStateConstPtr manipulator, 
			   bool isHighlighted = false, 
			   Eigen::Vector3f highlightedColor = Eigen::Vector3f(1.0, 0.0, 0.0));
	virtual ~OpenGL_Manipulator();

	//-------drawing
	virtual void draw();

	//observers
	ManipulatorStateConstPtr getManipulator() const;
};

} /* namespace affordance */
#endif /* OPENGL_MANIPULATOR_H */
